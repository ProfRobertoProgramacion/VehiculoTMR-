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
    // Definición del trapecio de entrada (ROI frontal proyectado en el piso)
    float y_top = img_h * params_.roi_top_pct;
    float y_bottom = img_h * params_.roi_bottom_pct;

    // Puntos fuente (trapecio en la imagen original)
    std::vector<cv::Point2f> src_pts = {
        cv::Point2f(img_w * 0.15f, y_bottom),  // Inferior Izquierda
        cv::Point2f(img_w * 0.85f, y_bottom),  // Inferior Derecha
        cv::Point2f(img_w * 0.65f, y_top),     // Superior Derecha
        cv::Point2f(img_w * 0.35f, y_top)      // Superior Izquierda
    };

    // Puntos destino (rectángulo en vista de pájaro Bird's Eye View)
    float bw = static_cast<float>(params_.bird_view_width);
    float bh = static_cast<float>(params_.bird_view_height);
    std::vector<cv::Point2f> dst_pts = {
        cv::Point2f(bw * 0.20f, bh),
        cv::Point2f(bw * 0.80f, bh),
        cv::Point2f(bw * 0.80f, 0.0f),
        cv::Point2f(bw * 0.20f, 0.0f)
    };

    perspective_matrix_ = cv::getPerspectiveTransform(src_pts, dst_pts);
    inv_perspective_matrix_ = cv::getPerspectiveTransform(dst_pts, src_pts);
}

cv::Mat LaneDetector::preprocess(const cv::Mat& bgr) {
    cv::Mat gray, blurred, binary;
    cv::cvtColor(bgr, gray, cv::COLOR_BGR2GRAY);
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 0);

    if (params_.use_otsu) {
        cv::threshold(blurred, binary, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    } else {
        cv::threshold(blurred, binary, params_.binary_threshold, 255, cv::THRESH_BINARY);
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

        int thresh_sum = 255 * (window_height / 3); // Mínimo de píxeles para considerar línea válida

        if (left_max > thresh_sum && right_max > thresh_sum) {
            // Se detectaron ambas líneas del carril
            found_center_x = (left_idx + right_idx) / 2;
            cv::circle(debug_vis, cv::Point(left_idx, mid_y), 4, cv::Scalar(255, 0, 0), -1);
            cv::circle(debug_vis, cv::Point(right_idx, mid_y), 4, cv::Scalar(0, 0, 255), -1);
        } else if (left_max > thresh_sum) {
            // Solo línea izquierda detectada: estimar centro sumando medio ancho de carril
            int lane_px = static_cast<int>(params_.lane_width_m / params_.pixels_to_meters);
            found_center_x = left_idx + (lane_px / 2);
            cv::circle(debug_vis, cv::Point(left_idx, mid_y), 4, cv::Scalar(255, 0, 0), -1);
        } else if (right_max > thresh_sum) {
            // Solo línea derecha detectada: estimar centro restando medio ancho de carril
            int lane_px = static_cast<int>(params_.lane_width_m / params_.pixels_to_meters);
            found_center_x = right_idx - (lane_px / 2);
            cv::circle(debug_vis, cv::Point(right_idx, mid_y), 4, cv::Scalar(0, 0, 255), -1);
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

    cv::Mat binary = preprocess(input_frame);
    cv::Mat warped = warpPerspective(binary);
    return extractLaneCenter(warped, debug_visualization);
}

} // namespace tmr_vision
