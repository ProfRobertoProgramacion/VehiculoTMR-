#ifndef TMR_CONTROL_LANE_CONTROLLER_HPP_
#define TMR_CONTROL_LANE_CONTROLLER_HPP_

#include <string>

namespace tmr_control {

struct ControllerParams {
    std::string type{"stanley"};        // "stanley" o "pid"

    // Parámetros Stanley
    double k_stanley{1.5};              // Ganancia de corrección lateral
    double epsilon_v{0.1};              // Evita división por cero a baja velocidad

    // Parámetros PID
    double kp{1.2};
    double ki{0.0};
    double kd{0.15};

    // Límites de Dirección (Ackermann)
    double max_steering_angle_rad{0.52}; // ~30 grados max

    // Control Longitudinal Adaptativo
    double v_min{0.35};                 // Velocidad mínima en curvas cerradas (m/s)
    double v_max{0.90};                 // Velocidad máxima en rectas (m/s)
    double k_curvature{2.0};            // Factor de desaceleración por curvatura
};

class LaneController {
public:
    LaneController();
    explicit LaneController(const ControllerParams& params);

    void setParams(const ControllerParams& params) { params_ = params; }
    const ControllerParams& getParams() const { return params_; }

    double computeSteering(double lateral_error, double heading_error, double current_speed, double dt);
    double computeSpeed(double curvature);

    void reset();

private:
    ControllerParams params_;
    double integral_error_{0.0};
    double prev_lateral_error_{0.0};
    bool first_pid_run_{true};
};

} // namespace tmr_control

#endif // TMR_CONTROL_LANE_CONTROLLER_HPP_
