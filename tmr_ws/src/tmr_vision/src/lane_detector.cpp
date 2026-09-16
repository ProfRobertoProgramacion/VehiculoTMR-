#include "tmr_vision/lane_detector.hpp"
#include <cmath>

namespace tmr_vision {

LaneDetector::LaneDetector() {
    updatePerspectiveMatrix(640, 480);
}

LaneDetector::LaneDetector(const VisionParams& params) : params_(params) {
    updatePerspectiveMatrix(640, 480);
}

void LaneDetector::setParams(const VisionParams& params) {
    params_ = params;
    updatePerspectiveMatrix(640, 480);
}

void LaneDetector::updatePerspectiveMatrix(int img_w, int img_h) {
    // Definición del trapecio frontal calibrado con medidas reales:
    // Base a 10 cm del auto: abarca 43 cm de ancho en el piso.
    // Carril oficial: 40 cm.
    float y_top = img_h * static_cast<float>(params_.roi_top_pct);
    float y_bottom = img_h * static_cast<float>(params_.roi_bottom_pct);

    // Puntos fuente (trapecio en imagen original 640x480)
    std::vector<cv::Point2f> src_pts = {
        cv::Point2f(img_w * 0.05f, y_bottom),  // Inferior Izquierda (~32 px)
        cv::Point2f(img_w * 0.95f, y_bottom),  // Inferior Derecha (~608 px)
        cv::Point2f(img_w * 0.74f, y_top),     // Superior Derecha (a 2 baldosas de profundidad)
        cv::Point2f(img_w * 0.26f, y_top)      // Superior Izquierda
    };

    // Puntos destino: Rectángulo métrico en Bird's Eye View (60 cm de ancho total x 60 cm de profundidad)
    float bw = static_cast<float>(params_.bird_view_width);
    float bh = static_cast<float>(params_.bird_view_height);
    // Carril de 40 cm centrado en ancho total de 60 cm: 67 px a cada margen lateral
    float lane_left_px = (0.60f - 0.40f) / 2.0f / 0.60f * bw;
    float lane_right_px = bw - lane_left_px;

    std::vector<cv::Point2f> dst_pts = {
        cv::Point2f(lane_left_px, bh),
        cv::Point2f(lane_right_px, bh),
        cv::Point2f(lane_right_px, 0.0f),
        cv::Point2f(lane_left_px, 0.0f)
    };

    perspective_matrix_ = cv::getPerspectiveTransform(src_pts, dst_pts);
    inv_perspective_matrix_ = cv::getPerspectiveTransform(dst_pts, src_pts);
}

cv::Mat LaneDetector::preprocess(const cv::Mat& bgr) {
    cv::Mat gray, warped_gray, blurred, binary;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);

    // 1. Transformación a Bird's Eye View (IPM) en escala de grises
    // Realizar la perspectiva en grises garantiza escala métrica uniforme (0.0015 m/px)
    cv::warpPerspective(gray, warped_gray, perspective_matrix_,
                        cv::Size(params_.bird_view_width, params_.bird_view_height));

    if (params_.filter_glare) {
        // Filtro Morfológico Top-Hat para eliminar reflejos de reflectores/lámparas sobre la lona negra del torneo.
        // En la vista IPM, la línea de 4 cm mide ~27 px. Un kernel horizontal de ~7 cm (47 px) elimina
        // manchas grandes de reflejo de lámparas del techo y conserva únicamente las franjas delgadas del carril.
        int k_width = static_cast<int>(0.07 / params_.pixels_to_meters);
        if (k_width % 2 == 0) k_width += 1;
        cv::Mat tophat_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(k_width, 3));
        cv::Mat tophat;
        cv::morphologyEx(warped_gray, tophat, cv::MORPH_TOPHAT, tophat_kernel);
        warped_gray = tophat;
    }

    cv::GaussianBlur(warped_gray, blurred, cv::Size(5, 5), 0);

    // Tipo de binarización:
    // invert_binary = true: cinta negra en piso blanco (entorno de prueba actual)
    // invert_binary = false: cinta blanca en lona negra (entorno oficial del torneo)
    int thresh_type = params_.invert_binary ? cv::THRESH_BINARY_INV : cv::THRESH_BINARY;

    if (params_.use_otsu) {
        cv::threshold(blurred, binary, 0, 255, thresh_type | cv::THRESH_OTSU);
    } else {
        cv::threshold(blurred, binary, params_.binary_threshold, 255, thresh_type);
    }

    // Operación morfológica para cerrar pequeños huecos en las líneas
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::morphologyEx(binary, binary, cv::MORPH_CLOSE, kernel);

    return binary;
}

cv::Mat LaneDetector::warpPerspective(const cv::Mat& binary) {
    cv::Mat warped;
    cv::warpPerspective(binary, warped, perspective_matrix_,
                        cv::Size(params_.bird_view_width, params_.bird_view_height));
    return warped;
}

LaneDetectionResult LaneDetector::extractLaneCenter(const cv::Mat& warped_binary, cv::Mat& debug_vis) {
    LaneDetectionResult result;
    int bw = warped_binary.cols;
    int bh = warped_binary.rows;
    int car_center_x = bw / 2;

    cv::cvtColor(warped_binary, debug_vis, cv::COLOR_GRAY2BGR);

    // Búsqueda de líneas en múltiples franjas horizontales (Sliding Windows)
    int n_windows = 8;
    int window_height = bh / n_windows;
    std::vector<int> left_centers;
    std::vector<int> right_centers;
    std::vector<int> center_points_x;
    std::vector<int> center_points_y;

    for (int i = 0; i < n_windows; ++i) {
        int win_y_bottom = bh - i * window_height;
        int win_y_top = win_y_bottom - window_height;
        if (win_y_top < 0) win_y_top = 0;

        // Histograma de la franja
        cv::Mat row_strip = warped_binary.rowRange(win_y_top, win_y_bottom);
        cv::Mat col_sum;
        cv::reduce(row_strip, col_sum, 0, cv::REDUCE_SUM, CV_32S);

        const int* sum_data = col_sum.ptr<int>(0);

        int left_max = 0, left_idx = -1;
        int right_max = 0, right_idx = -1;

        // Búsqueda en mitad izquierda
        for (int x = 10; x < car_center_x - 10; ++x) {
            if (sum_data[x] > left_max) {
                left_max = sum_data[x];
                left_idx = x;
            }
        }

        // Búsqueda en mitad derecha
        for (int x = car_center_x + 10; x < bw - 10; ++x) {
            if (sum_data[x] > right_max) {
                right_max = sum_data[x];
                right_idx = x;
            }
        }

        int mid_y = (win_y_top + win_y_bottom) / 2;
        int found_center_x = -1;

        int thresh_sum = 255 * (window_height / 4); // Mínimo de píxeles para considerar línea válida

        // Regla oficial TMR: mantener el carro a 4 cm del borde de la línea derecha
        // Distancia objetivo del centro del carro al CENTROIDE de la cinta derecha:
        // D = (car_width / 2) + target_right_margin + (line_thickness / 2)
        // Ejemplo prueba (cinta 2 cm): 0.09 + 0.04 + 0.01 = 0.14 m
        // Ejemplo torneo (cinta 4 cm): 0.09 + 0.04 + 0.02 = 0.15 m
        double target_dist_to_right_m = (params_.car_width_m / 2.0) + params_.target_right_margin_m + (params_.line_thickness_m / 2.0);
        int target_dist_px = static_cast<int>(target_dist_to_right_m / params_.pixels_to_meters);
        int lane_px = static_cast<int>(params_.lane_width_m / params_.pixels_to_meters);

        if (right_max > thresh_sum) {
            // Prioridad: Línea derecha detectada -> Ubicar el centro del auto respecto a la línea derecha
            found_center_x = right_idx - target_dist_px;
            cv::circle(debug_vis, cv::Point(right_idx, mid_y), 4, cv::Scalar(0, 0, 255), -1); // Línea derecha = ROJO

            if (left_max > thresh_sum) {
                cv::circle(debug_vis, cv::Point(left_idx, mid_y), 4, cv::Scalar(255, 0, 0), -1); // Línea izquierda = AZUL
                int lane_mid = (left_idx + right_idx) / 2;
                cv::circle(debug_vis, cv::Point(lane_mid, mid_y), 2, cv::Scalar(255, 255, 0), -1); // Centro carril = CIAN
            }
        } else if (left_max > thresh_sum) {
            // Si solo se ve la línea izquierda: proyectar hacia la derecha manteniendo el carril
            found_center_x = left_idx + (lane_px - target_dist_px);
            cv::circle(debug_vis, cv::Point(left_idx, mid_y), 4, cv::Scalar(255, 0, 0), -1);
        }

        if (found_center_x > 0 && found_center_x < bw) {
            center_points_x.push_back(found_center_x);
            center_points_y.push_back(mid_y);
            cv::circle(debug_vis, cv::Point(found_center_x, mid_y), 3, cv::Scalar(0, 255, 0), -1);
        }
    }

    // Dibujar línea central de referencia del auto
    cv::line(debug_vis, cv::Point(car_center_x, 0), cv::Point(car_center_x, bh), cv::Scalar(150, 150, 150), 1);

    if (center_points_x.size() >= 2) {
        result.valid = true;

        // Evaluar punto objetivo (Lookahead)
        int target_x = center_points_x[0];
        int target_y = center_points_y[0];

        // Buscar el punto más cercano a la distancia de lookahead deseada
        int min_dist = 9999;
        for (size_t i = 0; i < center_points_y.size(); ++i) {
            int d = std::abs(center_points_y[i] - params_.lookahead_y);
            if (d < min_dist) {
                min_dist = d;
                target_x = center_points_x[i];
                target_y = center_points_y[i];
            }
        }

        result.target_pixel_x = target_x;
        result.target_pixel_y = target_y;

        // 1. Error Lateral (metros): Distancia desde el centro del auto hasta el centro del carril
        // Si target_x > car_center_x, el carril está a la derecha del auto (el auto está a la izq)
        double delta_px = static_cast<double>(target_x - car_center_x);
        result.lateral_error_m = delta_px * params_.pixels_to_meters;

        // 2. Error de Orientación (radianes): Pendiente entre el punto base y el punto lookahead
        double dx = static_cast<double>(center_points_x.back() - center_points_x.front());
        double dy = static_cast<double>(center_points_y.front() - center_points_y.back()); // positivo hacia adelante
        result.heading_error_rad = std::atan2(dx, dy);

        // 3. Estimación de Curvatura (1/R)
        result.curvature = result.heading_error_rad / std::max(0.1, dy * params_.pixels_to_meters);

        // Visualización del objetivo
        cv::circle(debug_vis, cv::Point(target_x, target_y), 7, cv::Scalar(0, 255, 255), 2);
        cv::line(debug_vis, cv::Point(car_center_x, bh), cv::Point(target_x, target_y), cv::Scalar(0, 255, 255), 2);
    }

    return result;
}

LaneDetectionResult LaneDetector::process(const cv::Mat& input_frame, cv::Mat& debug_visualization) {
    if (input_frame.empty()) {
        return LaneDetectionResult();
    }

    cv::Mat warped_binary = preprocess(input_frame);
    return extractLaneCenter(warped_binary, debug_visualization);
}

} // namespace tmr_vision
