#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <cv_bridge/cv_bridge.h>
#include "tmr_vision/lane_detector.hpp"

class LaneDetectorNode : public rclcpp::Node {
public:
    LaneDetectorNode() : Node("lane_detector_node") {
        // Parámetros configurables
        this->declare_parameter<int>("binary_threshold", 200);
        this->declare_parameter<bool>("use_otsu", false);
        this->declare_parameter<double>("roi_top_pct", 0.45);
        this->declare_parameter<double>("roi_bottom_pct", 0.95);
        this->declare_parameter<double>("pixels_to_meters", 0.001);
        this->declare_parameter<double>("lane_width_m", 0.40);
        this->declare_parameter<int>("lookahead_y", 250);
        this->declare_parameter<bool>("use_compressed_sub", true);

        // Cargar parámetros en estructura
        tmr_vision::VisionParams params;
        params.binary_threshold = this->get_parameter("binary_threshold").as_int();
        params.use_otsu = this->get_parameter("use_otsu").as_bool();
        params.roi_top_pct = this->get_parameter("roi_top_pct").as_double();
        params.roi_bottom_pct = this->get_parameter("roi_bottom_pct").as_double();
        params.pixels_to_meters = this->get_parameter("pixels_to_meters").as_double();
        params.lane_width_m = this->get_parameter("lane_width_m").as_double();
        params.lookahead_y = this->get_parameter("lookahead_y").as_int();

        detector_.setParams(params);

        // Publicadores
        lane_error_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>(
            "/tmr/lane_error", 10);
        debug_img_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
            "/tmr/vision_debug", 10);

        bool use_compressed = this->get_parameter("use_compressed_sub").as_bool();

        if (use_compressed) {
            RCLCPP_INFO(this->get_logger(), "Suscribiéndose a imagen COMPRIMIDA (/camera/image_raw/compressed)");
            compressed_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
                "/camera/image_raw/compressed",
                rclcpp::SensorDataQoS(),
                std::bind(&LaneDetectorNode::compressedImageCallback, this, std::placeholders::_1));
        } else {
            RCLCPP_INFO(this->get_logger(), "Suscribiéndose a imagen RAW (/camera/image_raw)");
            raw_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
                "/camera/image_raw",
                10,
                std::bind(&LaneDetectorNode::rawImageCallback, this, std::placeholders::_1));
        }

        RCLCPP_INFO(this->get_logger(), "Nodo de visión inicializado para pista TMR.");
    }

private:
    void compressedImageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
        cv::Mat bgr_frame;
        try {
            bgr_frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error decodificando imagen comprimida: %s", e.what());
            return;
        }

        if (!bgr_frame.empty()) {
            processAndPublish(bgr_frame, msg->header);
        }
    }

    void rawImageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error en cv_bridge: %s", e.what());
            return;
        }

        if (cv_ptr && !cv_ptr->image.empty()) {
            processAndPublish(cv_ptr->image, msg->header);
        }
    }

    void processAndPublish(const cv::Mat& frame, const std_msgs::msg::Header& header) {
        auto start_time = std::chrono::steady_clock::now();

        cv::Mat debug_img;
        auto res = detector_.process(frame, debug_img);

        auto end_time = std::chrono::steady_clock::now();
        double elapsed_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        if (res.valid) {
            geometry_msgs::msg::Pose2D err_msg;
            err_msg.x = res.lateral_error_m;       // Error lateral e_lat (m)
            err_msg.theta = res.heading_error_rad; // Error angular e_heading (rad)
            err_msg.y = res.curvature;             // Curvatura estimada kappa (1/m)
            lane_error_pub_->publish(err_msg);
        }

        // Publicar imagen de depuración
        if (debug_img_pub_->get_subscription_count() > 0 && !debug_img.empty()) {
            cv_bridge::CvImage debug_cv_img(header, "bgr8", debug_img);
            debug_img_pub_->publish(*debug_cv_img.toImageMsg());
        }

        RCLCPP_DEBUG(this->get_logger(), "Visión procesada en %.2f ms (Válida: %d, e_lat: %.3f m)",
                     elapsed_ms, res.valid, res.lateral_error_m);
    }

    tmr_vision::LaneDetector detector_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr lane_error_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_img_pub_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr raw_sub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneDetectorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
