#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include "tmr_control/lane_controller.hpp"

using namespace std::chrono_literals;

class LaneControllerNode : public rclcpp::Node {
public:
    LaneControllerNode() : Node("lane_controller_node") {
        // Parámetros
        this->declare_parameter<std::string>("controller_type", "stanley");
        this->declare_parameter<double>("k_stanley", 1.5);
        this->declare_parameter<double>("epsilon_v", 0.1);
        this->declare_parameter<double>("kp", 1.2);
        this->declare_parameter<double>("ki", 0.0);
        this->declare_parameter<double>("kd", 0.15);
        this->declare_parameter<double>("max_steering_angle_rad", 0.52);
        this->declare_parameter<double>("v_min", 0.35);
        this->declare_parameter<double>("v_max", 0.90);
        this->declare_parameter<double>("k_curvature", 2.0);
        this->declare_parameter<bool>("autonomous_enabled_by_default", true);

        tmr_control::ControllerParams p;
        p.type = this->get_parameter("controller_type").as_string();
        p.k_stanley = this->get_parameter("k_stanley").as_double();
        p.epsilon_v = this->get_parameter("epsilon_v").as_double();
        p.kp = this->get_parameter("kp").as_double();
        p.ki = this->get_parameter("ki").as_double();
        p.kd = this->get_parameter("kd").as_double();
        p.max_steering_angle_rad = this->get_parameter("max_steering_angle_rad").as_double();
        p.v_min = this->get_parameter("v_min").as_double();
        p.v_max = this->get_parameter("v_max").as_double();
        p.k_curvature = this->get_parameter("k_curvature").as_double();

        controller_.setParams(p);
        autonomous_enabled_ = this->get_parameter("autonomous_enabled_by_default").as_bool();

        // Publicadores y subscriptores
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        lane_error_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "/tmr/lane_error", 10,
            std::bind(&LaneControllerNode::laneErrorCallback, this, std::placeholders::_1));

        enable_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/tmr/enable_auto", 10,
            [this](const std_msgs::msg::Bool::SharedPtr msg) {
                autonomous_enabled_ = msg->data;
                RCLCPP_INFO(this->get_logger(), "Modo Autónomo: %s", autonomous_enabled_ ? "ACTIVADO" : "DETENIDO");
                if (!autonomous_enabled_) {
                    stopVehicle();
                }
            });

        // Watchdog por pérdida de visión (detener el vehículo si no hay línea)
        last_error_time_ = this->now();
        watchdog_timer_ = this->create_wall_timer(
            100ms, std::bind(&LaneControllerNode::checkWatchdog, this));

        RCLCPP_INFO(this->get_logger(), "Nodo de control iniciado (Controlador: %s, V_max: %.2f m/s)",
                    p.type.c_str(), p.v_max);
    }

private:
    void laneErrorCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
        if (!autonomous_enabled_) {
            return;
        }

        rclcpp::Time now = this->now();
        double dt = (now - last_error_time_).seconds();
        last_error_time_ = now;

        double e_lat = msg->x;
        double e_heading = msg->theta;
        double curvature = msg->y;

        // Calcular velocidad adaptativa según la curvatura de la pista
        double target_v = controller_.computeSpeed(curvature);

        // Calcular ángulo de dirección (Stanley o PID)
        double steering_angle = controller_.computeSteering(e_lat, e_heading, target_v, dt);

        // Publicar comando de velocidad
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = target_v;          // m/s
        cmd.angular.z = steering_angle;   // rad (ángulo de dirección de ruedas delanteras)
        cmd_vel_pub_->publish(cmd);

        RCLCPP_DEBUG(this->get_logger(), "Control -> V: %.2f m/s, Steer: %.2f deg",
                     target_v, steering_angle * 180.0 / M_PI);
    }

    void checkWatchdog() {
        if (!autonomous_enabled_) return;

        double elapsed = (this->now() - last_error_time_).seconds();
        if (elapsed > 0.4) { // Si pasan más de 400ms sin ver el carril
            stopVehicle();
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "Watchdog de visión activado: sin detección de carril. Auto detenido.");
        }
    }

    void stopVehicle() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
        controller_.reset();
    }

    tmr_control::LaneController controller_;
    bool autonomous_enabled_{true};
    rclcpp::Time last_error_time_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr lane_error_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_sub_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LaneControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
