#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;

template <typename T>
static inline T clamp_val(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}

class ArduinoBridgeNode : public rclcpp::Node {
public:
    ArduinoBridgeNode() : Node("arduino_bridge_node"), serial_fd_(-1) {
        // Parámetros de puerto serie
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baud_rate", 115200);

        // Calibración de Servo (Dirección)
        this->declare_parameter<int>("servo_center_deg", 90);
        this->declare_parameter<int>("servo_range_deg", 35);
        this->declare_parameter<double>("max_steering_angle_rad", 0.52);
        this->declare_parameter<bool>("invert_steering", false);

        // Calibración de ESC (Tracción)
        this->declare_parameter<int>("esc_neutral_us", 1500);
        this->declare_parameter<int>("esc_forward_min_us", 1540); // Umbral mínimo de movimiento
        this->declare_parameter<int>("esc_forward_max_us", 1650); // Velocidad máxima segura
        this->declare_parameter<double>("v_max_mps", 1.0);

        port_ = this->get_parameter("port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        servo_center_deg_ = this->get_parameter("servo_center_deg").as_int();
        servo_range_deg_ = this->get_parameter("servo_range_deg").as_int();
        max_steering_angle_rad_ = this->get_parameter("max_steering_angle_rad").as_double();
        invert_steering_ = this->get_parameter("invert_steering").as_bool();

        esc_neutral_us_ = this->get_parameter("esc_neutral_us").as_int();
        esc_forward_min_us_ = this->get_parameter("esc_forward_min_us").as_int();
        esc_forward_max_us_ = this->get_parameter("esc_forward_max_us").as_int();
        v_max_mps_ = this->get_parameter("v_max_mps").as_double();

        // Inicializar puerto serie
        initSerial();

        // Subscripción a comandos de velocidad
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&ArduinoBridgeNode::cmdCallback, this, std::placeholders::_1));

        // Timer de reintento de conexión y heartbeat
        heartbeat_timer_ = this->create_wall_timer(
            100ms, std::bind(&ArduinoBridgeNode::heartbeat, this));

        RCLCPP_INFO(this->get_logger(), "Arduino Bridge iniciado en %s a %d baudios", port_.c_str(), baud_rate_);
    }

    ~ArduinoBridgeNode() override {
        if (serial_fd_ >= 0) {
            sendCommand(servo_center_deg_, esc_neutral_us_);
            close(serial_fd_);
        }
    }

private:
    void initSerial() {
        if (serial_fd_ >= 0) {
            close(serial_fd_);
            serial_fd_ = -1;
        }

        serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) {
            // Auto-detección: si el puerto configurado no abre, probar alternativas comunes
            const std::vector<std::string> candidates = {"/dev/ttyUSB0", "/dev/ttyACM0", "/dev/ttyUSB1", "/dev/ttyACM1"};
            for (const auto &cand : candidates) {
                if (cand != port_ && access(cand.c_str(), F_OK) == 0) {
                    RCLCPP_INFO(this->get_logger(), "Puerto %s no disponible. Auto-detectado y probando %s...",
                                port_.c_str(), cand.c_str());
                    port_ = cand;
                    serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
                    if (serial_fd_ >= 0) {
                        RCLCPP_INFO(this->get_logger(), "¡Conectado exitosamente al puerto alternativo %s!", port_.c_str());
                        break;
                    }
                }
            }
        }

        if (serial_fd_ < 0) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "No se puede abrir el puerto %s. Reintentando...", port_.c_str());
            return;
        }

        struct termios tty;
        if (tcgetattr(serial_fd_, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error obteniendo atributos termios para %s", port_.c_str());
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        speed_t speed = B115200;
        if (baud_rate_ == 9600) speed = B9600;
        else if (baud_rate_ == 57600) speed = B57600;

        cfsetospeed(&tty, speed);
        cfsetispeed(&tty, speed);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8 bits
        tty.c_cflag |= (CLOCAL | CREAD);            // Habilitar lectura
        tty.c_cflag &= ~(PARENB | PARODD);          // Sin paridad
        tty.c_cflag &= ~CSTOPB;                     // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;                    // Sin control de flujo por hardware

        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
        tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
        tty.c_oflag &= ~OPOST;

        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1; // Timeout de lectura de 100ms

        if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Error aplicando configuración termios");
            close(serial_fd_);
            serial_fd_ = -1;
            return;
        }

        // Dar un segundo al Arduino para resetearse tras abrir DTR
        sleep(1);
        tcflush(serial_fd_, TCIOFLUSH);
        RCLCPP_INFO(this->get_logger(), "Puerto serie %s conectado y configurado exitosamente.", port_.c_str());
    }

    void cmdCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_cmd_time_ = this->now();

        // 1. Mapeo de dirección: angular.z (rad) -> servo_deg
        double steer_rad = msg->angular.z;
        double steer_normalized = clamp_val(steer_rad / max_steering_angle_rad_, -1.0, 1.0);

        if (invert_steering_) {
            steer_normalized = -steer_normalized;
        }

        // steer_normalized: +1 = izq, -1 = der
        int target_servo = servo_center_deg_ - static_cast<int>(steer_normalized * servo_range_deg_);
        target_servo = clamp_val(target_servo, servo_center_deg_ - servo_range_deg_,
                                               servo_center_deg_ + servo_range_deg_);

        // 2. Mapeo de aceleración: linear.x (m/s) -> esc_us
        int target_esc = esc_neutral_us_;
        double v = msg->linear.x;

        if (v > 0.01) {
            double v_ratio = clamp_val(v / v_max_mps_, 0.0, 1.0);
            target_esc = esc_forward_min_us_ + static_cast<int>(v_ratio * (esc_forward_max_us_ - esc_forward_min_us_));
        } else if (v < -0.01) {
            // Reversa suave (opcional)
            target_esc = esc_neutral_us_ - 60;
        } else {
            target_esc = esc_neutral_us_;
        }

        current_servo_ = target_servo;
        current_esc_ = target_esc;

        sendCommand(current_servo_, current_esc_);
    }

    void heartbeat() {
        if (serial_fd_ < 0) {
            initSerial();
            return;
        }

        // Watchdog local: Si no se han recibido comandos de ROS en 500ms, enviar orden de detención
        double elapsed = (this->now() - last_cmd_time_).seconds();
        if (elapsed > 0.5 && current_esc_ != esc_neutral_us_) {
            current_esc_ = esc_neutral_us_;
            sendCommand(current_servo_, current_esc_);
        }
    }

    void sendCommand(int steering_deg, int throttle_us) {
        if (serial_fd_ < 0) return;

        char buf[32];
        int len = snprintf(buf, sizeof(buf), "<%d,%d>\n", steering_deg, throttle_us);
        if (len > 0) {
            ssize_t bytes_written = write(serial_fd_, buf, len);
            if (bytes_written < 0) {
                RCLCPP_WARN(this->get_logger(), "Fallo escribiendo al puerto serie. Reiniciando conexión.");
                close(serial_fd_);
                serial_fd_ = -1;
            }
        }
    }

    std::string port_{"/dev/ttyACM0"};
    int baud_rate_{115200};
    int servo_center_deg_{90};
    int servo_range_deg_{35};
    double max_steering_angle_rad_{0.52};
    bool invert_steering_{false};

    int esc_neutral_us_{1500};
    int esc_forward_min_us_{1540};
    int esc_forward_max_us_{1650};
    double v_max_mps_{1.0};

    int current_servo_{90};
    int current_esc_{1500};

    int serial_fd_{-1};
    rclcpp::Time last_cmd_time_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::TimerBase::SharedPtr heartbeat_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ArduinoBridgeNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
