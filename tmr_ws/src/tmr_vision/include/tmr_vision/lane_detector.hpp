#ifndef TMR_VISION_LANE_DETECTOR_HPP_
#define TMR_VISION_LANE_DETECTOR_HPP_

#include <opencv2/opencv.hpp>
#include <vector>

namespace tmr_vision {

struct LaneDetectionResult {
    bool valid{false};
    double lateral_error_m{0.0};    // Error lateral en metros (+: auto a la izq del centro, -: a la der)
    double heading_error_rad{0.0};  // Error de ángulo con respecto al carril (+: apunta a la izq, -: a la der)
    double curvature{0.0};          // Estimación de curvatura (1/radio)
    int target_pixel_x{0};
    int target_pixel_y{0};
};

struct VisionParams {
    int binary_threshold{110};      // Umbral de binarización
    bool use_otsu{false};           // Usar Otsu adaptativo
    bool invert_binary{true};       // true: cinta negra en piso blanco (prueba), false: cinta blanca en lona negra (torneo)
    bool filter_glare{false};       // Filtro Morfológico Top-Hat para eliminar reflejos de lámparas en lona negra de torneo
    double roi_top_pct{0.42};       // Horizonte (ignorar fondo y pared)
    double roi_bottom_pct{0.98};    // Base inferior (a 10 cm del frente del auto)
    int bird_view_width{400};       // Ancho resolución IPM
    int bird_view_height{400};      // Alto resolución IPM
    double pixels_to_meters{0.0015}; // Factor de escala: 0.60 m / 400 px = 1.5 mm/pixel
    double lane_width_m{0.40};      // Ancho carril oficial TMR: 40 cm
    double car_width_m{0.18};       // Ancho del auto: 18 cm
    double target_right_margin_m{0.04}; // Margen oficial a la línea derecha: 4 cm
    double line_thickness_m{0.02};  // Grosor de cinta: 2 cm en prueba, 4 cm en torneo
    int lookahead_y{250};           // Fila Y de evaluación de dirección en la vista IPM
};

class LaneDetector {
public:
    LaneDetector();
    explicit LaneDetector(const VisionParams& params);

    void setParams(const VisionParams& params);
    const VisionParams& getParams() const { return params_; }

    LaneDetectionResult process(const cv::Mat& input_frame, cv::Mat& debug_visualization);

private:
    VisionParams params_;
    cv::Mat perspective_matrix_;
    cv::Mat inv_perspective_matrix_;

    void updatePerspectiveMatrix(int img_w, int img_h);
    cv::Mat preprocess(const cv::Mat& bgr);
    cv::Mat warpPerspective(const cv::Mat& binary);
    LaneDetectionResult extractLaneCenter(const cv::Mat& warped_binary, cv::Mat& debug_vis);
};

} // namespace tmr_vision

#endif // TMR_VISION_LANE_DETECTOR_HPP_
