#include "tmr_control/lane_controller.hpp"
#include <cmath>
#include <algorithm>

template <typename T>
static inline T clamp_val(T val, T min_val, T max_val) {
    return std::max(min_val, std::min(val, max_val));
}

namespace tmr_control {

LaneController::LaneController() : params_() {}

LaneController::LaneController(const ControllerParams& params) : params_(params) {}

void LaneController::reset() {
    integral_error_ = 0.0;
    prev_lateral_error_ = 0.0;
    first_pid_run_ = true;
}

double LaneController::computeSteering(double lateral_error, double heading_error, double current_speed, double dt) {
    double steering = 0.0;

    if (params_.type == "stanley") {
        // Controlador Stanley:
        // delta = - (heading_error + arctan(k * lateral_error / (v + epsilon)))
        double effective_speed = std::max(0.05, std::abs(current_speed)) + params_.epsilon_v;
        double cross_track_term = std::atan2(params_.k_stanley * lateral_error, effective_speed);
        steering = -(heading_error + cross_track_term);
    } else {
        // Controlador PID sobre el error lateral
        if (first_pid_run_) {
            prev_lateral_error_ = lateral_error;
            first_pid_run_ = false;
        }

        double p_term = params_.kp * lateral_error;

        if (dt > 0.0) {
            integral_error_ += lateral_error * dt;
            // Anti-windup
            integral_error_ = clamp_val(integral_error_, -0.5, 0.5);
        }
        double i_term = params_.ki * integral_error_;

        double derivative = (dt > 0.0) ? (lateral_error - prev_lateral_error_) / dt : 0.0;
        prev_lateral_error_ = lateral_error;
        double d_term = params_.kd * derivative;

        steering = -(p_term + i_term + d_term);
    }

    // Limitar al rango físico de la dirección Ackermann
    steering = clamp_val(steering, -params_.max_steering_angle_rad, params_.max_steering_angle_rad);
    return steering;
}

double LaneController::computeSpeed(double curvature) {
    // Reducción adaptativa de velocidad en función de la curvatura de la pista
    double abs_curv = std::abs(curvature);
    double target_speed = params_.v_max / (1.0 + params_.k_curvature * abs_curv);
    return clamp_val(target_speed, params_.v_min, params_.v_max);
}

} // namespace tmr_control
