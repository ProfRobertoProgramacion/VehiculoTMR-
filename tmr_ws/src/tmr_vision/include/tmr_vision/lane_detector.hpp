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
    int binary_threshold{200};      // Umbral para líneas blancas sobre pista negra
    bool use_otsu{false};           // Usar umbralizado adaptativo Otsu
    double roi_top_pct{0.45};       // Recorte de horizonte (ignorar parte superior de la imagen)
    double roi_bottom_pct{0.95};
    int bird_view_width{400};
    int bird_view_height{400};
    double pixels_to_meters{0.001}; // Factor de conversión escala píxel a metro
    double lane_width_m{0.40};      // Ancho oficial carril TMR (40 cm)
    int lookahead_y{250};           // Fila Y de evaluación en la vista de pájaro
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
