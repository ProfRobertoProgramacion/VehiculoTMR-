#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <chrono>

using namespace std::chrono_literals;

class UsbCameraNode : public rclcpp::Node {
public:
    UsbCameraNode() : Node("usb_camera_node") {
        // Declaración de parámetros configurables
        this->declare_parameter<int>("device_id", 0);
        this->declare_parameter<int>("width", 640);
        this->declare_parameter<int>("height", 480);
        this->declare_parameter<int>("framerate", 30);
        this->declare_parameter<int>("jpeg_quality", 75);
        this->declare_parameter<std::string>("frame_id", "camera_link");
        this->declare_parameter<bool>("publish_raw", true);

        // Obtención de parámetros
        device_id_ = this->get_parameter("device_id").as_int();
        width_ = this->get_parameter("width").as_int();
        height_ = this->get_parameter("height").as_int();
        framerate_ = this->get_parameter("framerate").as_int();
        jpeg_quality_ = this->get_parameter("jpeg_quality").as_int();
        frame_id_ = this->get_parameter("frame_id").as_string();
        publish_raw_ = this->get_parameter("publish_raw").as_bool();

        RCLCPP_INFO(this->get_logger(), "Iniciando cámara USB en /dev/video%d (%dx%d @ %d FPS)",
                    device_id_, width_, height_, framerate_);

        // Publicadores
        compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "/camera/image_raw/compressed", rclcpp::SensorDataQoS());

        if (publish_raw_) {
            raw_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
                "/camera/image_raw", 10);
        }

        // Inicializar captura OpenCV
        cap_.open(device_id_, cv::CAP_V4L2);
        if (!cap_.isOpened()) {
            RCLCPP_WARN(this->get_logger(), "No se pudo abrir con CAP_V4L2, intentando con backend por defecto...");
            cap_.open(device_id_);
        }

        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "¡ERROR CRÍTICO! No se pudo abrir la cámara USB en /dev/video%d", device_id_);
            return;
        }

        // Configurar propiedades de la cámara
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
        cap_.set(cv::CAP_PROP_FPS, framerate_);
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        // Parámetros para compresión JPEG eficiente
        compression_params_.push_back(cv::IMWRITE_JPEG_QUALITY);
        compression_params_.push_back(jpeg_quality_);

        // Timer de captura a la tasa especificada
        std::chrono::milliseconds interval(1000 / std::max(1, framerate_));
        timer_ = this->create_wall_timer(interval, std::bind(&UsbCameraNode::captureFrame, this));

        RCLCPP_INFO(this->get_logger(), "Cámara USB iniciada exitosamente.");
    }

    ~UsbCameraNode() override {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

private:
    void captureFrame() {
        if (!cap_.isOpened()) return;

        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "No se pudo leer cuadro de la cámara USB");
            return;
        }

        auto timestamp = this->now();

        // 1. Publicar imagen Comprimida (esencial para transmisión Wi-Fi hacia la PC con baja latencia)
        std::vector<uchar> buffer;
        if (cv::imencode(".jpg", frame, buffer, compression_params_)) {
            sensor_msgs::msg::CompressedImage comp_msg;
            comp_msg.header.stamp = timestamp;
            comp_msg.header.frame_id = frame_id_;
            comp_msg.format = "jpeg";
            comp_msg.data = std::move(buffer);
            compressed_pub_->publish(comp_msg);
        }

        // 2. Publicar imagen sin comprimir (para uso local en el mismo host)
        if (publish_raw_ && raw_pub_) {
            std_msgs::msg::Header header;
            header.stamp = timestamp;
            header.frame_id = frame_id_;
            cv_bridge::CvImage cv_img(header, "bgr8", frame);
            raw_pub_->publish(*cv_img.toImageMsg());
        }
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr raw_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int device_id_{0};
    int width_{640};
    int height_{480};
    int framerate_{30};
    int jpeg_quality_{75};
    std::string frame_id_{"camera_link"};
    bool publish_raw_{true};
    std::vector<int> compression_params_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<UsbCameraNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
