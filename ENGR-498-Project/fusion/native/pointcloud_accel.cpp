#include <cmath>
#include <cstdint>

#if defined(_WIN32)
#define FUSION_EXPORT extern "C" __declspec(dllexport)
#else
#define FUSION_EXPORT extern "C"
#endif

namespace {

constexpr double kMinDepth = 1e-9;

inline void transform_point(
    const double* matrix,
    double x,
    double y,
    double z,
    double& out_x,
    double& out_y,
    double& out_z) {
    out_x = matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3];
    out_y = matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7];
    out_z = matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11];
}

inline void apply_distortion(
    const double* distortion,
    int32_t distortion_len,
    double x,
    double y,
    double& out_x,
    double& out_y) {
    double k1 = 0.0;
    double k2 = 0.0;
    double p1 = 0.0;
    double p2 = 0.0;
    double k3 = 0.0;
    double k4 = 0.0;
    double k5 = 0.0;
    double k6 = 0.0;

    if (distortion_len > 0) { k1 = distortion[0]; }
    if (distortion_len > 1) { k2 = distortion[1]; }
    if (distortion_len > 2) { p1 = distortion[2]; }
    if (distortion_len > 3) { p2 = distortion[3]; }
    if (distortion_len > 4) { k3 = distortion[4]; }
    if (distortion_len > 5) { k4 = distortion[5]; }
    if (distortion_len > 6) { k5 = distortion[6]; }
    if (distortion_len > 7) { k6 = distortion[7]; }

    const double r2 = x * x + y * y;
    const double r4 = r2 * r2;
    const double r6 = r4 * r2;

    double radial = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
    if (distortion_len > 5) {
        const double denominator = 1.0 + k4 * r2 + k5 * r4 + k6 * r6;
        if (std::abs(denominator) > kMinDepth) {
            radial /= denominator;
        }
    }

    const double xy2 = 2.0 * x * y;
    const double x2 = x * x;
    const double y2 = y * y;

    out_x = x * radial + p1 * xy2 + p2 * (r2 + 2.0 * x2);
    out_y = y * radial + p1 * (r2 + 2.0 * y2) + p2 * xy2;
}

}  // namespace

FUSION_EXPORT int project_assign_best_detection(
    const double* points_map,
    int64_t n_points,
    const double* map_to_lidar,
    const double* lidar_to_cam,
    double fx,
    double fy,
    double cx,
    double cy,
    int32_t image_width,
    int32_t image_height,
    const double* distortion,
    int32_t distortion_len,
    const uint8_t* masks,
    int32_t num_instances,
    const double* confidences,
    const uint8_t* allowed,
    int32_t* out_detection_index,
    double* out_detection_confidence) {
    if (points_map == nullptr || map_to_lidar == nullptr || lidar_to_cam == nullptr ||
        masks == nullptr || confidences == nullptr || allowed == nullptr ||
        out_detection_index == nullptr || out_detection_confidence == nullptr ||
        n_points < 0 || image_width <= 0 || image_height <= 0 || num_instances < 0) {
        return 1;
    }

    const int64_t pixels_per_mask = static_cast<int64_t>(image_width) * image_height;

#if defined(_OPENMP)
#pragma omp parallel for schedule(static)
#endif
    for (int64_t idx = 0; idx < n_points; ++idx) {
        out_detection_index[idx] = -1;
        out_detection_confidence[idx] = 0.0;

        const double x_map = points_map[idx * 3 + 0];
        const double y_map = points_map[idx * 3 + 1];
        const double z_map = points_map[idx * 3 + 2];

        double x_lidar = 0.0;
        double y_lidar = 0.0;
        double z_lidar = 0.0;
        transform_point(map_to_lidar, x_map, y_map, z_map, x_lidar, y_lidar, z_lidar);

        double x_cam = 0.0;
        double y_cam = 0.0;
        double z_cam = 0.0;
        transform_point(lidar_to_cam, x_lidar, y_lidar, z_lidar, x_cam, y_cam, z_cam);

        if (z_cam <= kMinDepth) {
            continue;
        }

        const double xn = x_cam / z_cam;
        const double yn = y_cam / z_cam;

        double xd = xn;
        double yd = yn;
        if (distortion != nullptr && distortion_len > 0) {
            apply_distortion(distortion, distortion_len, xn, yn, xd, yd);
        }

        const int32_t u = static_cast<int32_t>(std::llround(fx * xd + cx));
        const int32_t v = static_cast<int32_t>(std::llround(fy * yd + cy));

        if (u < 0 || u >= image_width || v < 0 || v >= image_height) {
            continue;
        }

        double best_conf = 0.0;
        int32_t best_detection = -1;
        const int64_t pixel_index = static_cast<int64_t>(v) * image_width + u;

        for (int32_t det_idx = 0; det_idx < num_instances; ++det_idx) {
            if (allowed[det_idx] == 0) {
                continue;
            }

            const int64_t mask_index = static_cast<int64_t>(det_idx) * pixels_per_mask + pixel_index;
            if (masks[mask_index] == 0) {
                continue;
            }

            const double confidence = confidences[det_idx];
            if (confidence > best_conf) {
                best_conf = confidence;
                best_detection = det_idx;
            }
        }

        out_detection_index[idx] = best_detection;
        out_detection_confidence[idx] = best_conf;
    }

    return 0;
}
