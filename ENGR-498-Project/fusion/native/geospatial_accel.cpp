#include <cstdint>

#if defined(_WIN32)
#define FUSION_EXPORT extern "C" __declspec(dllexport)
#else
#define FUSION_EXPORT extern "C"
#endif

FUSION_EXPORT int apply_similarity_xyz(
    const double* points_in,
    int64_t n_points,
    double scale,
    double cos_yaw,
    double sin_yaw,
    double tx,
    double ty,
    double tz,
    double* points_out) {
    if (points_in == nullptr || points_out == nullptr || n_points < 0) {
        return 1;
    }

#if defined(_OPENMP)
#pragma omp parallel for schedule(static)
#endif
    for (int64_t idx = 0; idx < n_points; ++idx) {
        const double x = points_in[idx * 3 + 0];
        const double y = points_in[idx * 3 + 1];
        const double z = points_in[idx * 3 + 2];

        points_out[idx * 3 + 0] = scale * (cos_yaw * x - sin_yaw * y) + tx;
        points_out[idx * 3 + 1] = scale * (sin_yaw * x + cos_yaw * y) + ty;
        points_out[idx * 3 + 2] = z + tz;
    }

    return 0;
}
