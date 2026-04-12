#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

#if defined(_WIN32)
#define WIRE_EXPORT extern "C" __declspec(dllexport)
#else
#define WIRE_EXPORT extern "C"
#endif

namespace {

constexpr double kEpsilon = 1e-12;

void jacobi_eigen_decomposition(double a[3][3], double v[3][3]) {
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            v[row][col] = (row == col) ? 1.0 : 0.0;
        }
    }

    for (int iter = 0; iter < 16; ++iter) {
        int p = 0;
        int q = 1;
        double max_off_diag = std::abs(a[0][1]);

        if (std::abs(a[0][2]) > max_off_diag) {
            p = 0;
            q = 2;
            max_off_diag = std::abs(a[0][2]);
        }
        if (std::abs(a[1][2]) > max_off_diag) {
            p = 1;
            q = 2;
            max_off_diag = std::abs(a[1][2]);
        }

        if (max_off_diag < kEpsilon) {
            break;
        }

        const double app = a[p][p];
        const double aqq = a[q][q];
        const double apq = a[p][q];
        const double tau = (aqq - app) / (2.0 * apq);
        const double t = (tau >= 0.0)
            ? 1.0 / (tau + std::sqrt(1.0 + tau * tau))
            : -1.0 / (-tau + std::sqrt(1.0 + tau * tau));
        const double c = 1.0 / std::sqrt(1.0 + t * t);
        const double s = t * c;

        a[p][p] = app - t * apq;
        a[q][q] = aqq + t * apq;
        a[p][q] = 0.0;
        a[q][p] = 0.0;

        for (int r = 0; r < 3; ++r) {
            if (r == p || r == q) {
                continue;
            }
            const double arp = a[r][p];
            const double arq = a[r][q];
            a[r][p] = c * arp - s * arq;
            a[p][r] = a[r][p];
            a[r][q] = c * arq + s * arp;
            a[q][r] = a[r][q];
        }

        for (int r = 0; r < 3; ++r) {
            const double vrp = v[r][p];
            const double vrq = v[r][q];
            v[r][p] = c * vrp - s * vrq;
            v[r][q] = c * vrq + s * vrp;
        }
    }
}

}  // namespace

WIRE_EXPORT int compute_local_principal_features(
    const double* points,
    int64_t n_points,
    const int64_t* neighbor_indices,
    const int64_t* neighbor_offsets,
    int64_t n_queries,
    double* out_principal_dirs,
    double* out_linearness,
    uint8_t* out_valid) {
    if (points == nullptr || neighbor_indices == nullptr || neighbor_offsets == nullptr ||
        out_principal_dirs == nullptr || out_linearness == nullptr || out_valid == nullptr ||
        n_points < 0 || n_queries < 0) {
        return 1;
    }

#if defined(_OPENMP)
#pragma omp parallel for schedule(static)
#endif
    for (int64_t query_idx = 0; query_idx < n_queries; ++query_idx) {
        out_valid[query_idx] = 0;
        out_linearness[query_idx] = 0.0;
        out_principal_dirs[query_idx * 3 + 0] = 0.0;
        out_principal_dirs[query_idx * 3 + 1] = 0.0;
        out_principal_dirs[query_idx * 3 + 2] = 0.0;

        const int64_t begin = neighbor_offsets[query_idx];
        const int64_t end = neighbor_offsets[query_idx + 1];
        const int64_t count = end - begin;
        if (count < 3) {
            continue;
        }

        double mean_x = 0.0;
        double mean_y = 0.0;
        double mean_z = 0.0;
        int64_t valid_neighbors = 0;

        for (int64_t offset = begin; offset < end; ++offset) {
            const int64_t point_idx = neighbor_indices[offset];
            if (point_idx < 0 || point_idx >= n_points) {
                continue;
            }

            mean_x += points[point_idx * 3 + 0];
            mean_y += points[point_idx * 3 + 1];
            mean_z += points[point_idx * 3 + 2];
            ++valid_neighbors;
        }

        if (valid_neighbors < 3) {
            continue;
        }

        const double inv_count = 1.0 / static_cast<double>(valid_neighbors);
        mean_x *= inv_count;
        mean_y *= inv_count;
        mean_z *= inv_count;

        double covariance[3][3] = {
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
        };

        for (int64_t offset = begin; offset < end; ++offset) {
            const int64_t point_idx = neighbor_indices[offset];
            if (point_idx < 0 || point_idx >= n_points) {
                continue;
            }

            const double dx = points[point_idx * 3 + 0] - mean_x;
            const double dy = points[point_idx * 3 + 1] - mean_y;
            const double dz = points[point_idx * 3 + 2] - mean_z;

            covariance[0][0] += dx * dx;
            covariance[0][1] += dx * dy;
            covariance[0][2] += dx * dz;
            covariance[1][1] += dy * dy;
            covariance[1][2] += dy * dz;
            covariance[2][2] += dz * dz;
        }

        const double scale = 1.0 / static_cast<double>(valid_neighbors - 1);
        covariance[0][0] *= scale;
        covariance[0][1] *= scale;
        covariance[0][2] *= scale;
        covariance[1][1] *= scale;
        covariance[1][2] *= scale;
        covariance[2][2] *= scale;
        covariance[1][0] = covariance[0][1];
        covariance[2][0] = covariance[0][2];
        covariance[2][1] = covariance[1][2];

        double eigenvectors[3][3];
        jacobi_eigen_decomposition(covariance, eigenvectors);

        std::array<double, 3> eigenvalues = {
            covariance[0][0],
            covariance[1][1],
            covariance[2][2],
        };
        std::array<int, 3> order = {0, 1, 2};
        std::sort(order.begin(), order.end(), [&](int lhs, int rhs) {
            return eigenvalues[lhs] > eigenvalues[rhs];
        });

        const double lambda1 = eigenvalues[order[0]];
        const double lambda2 = eigenvalues[order[1]];
        if (!(lambda1 > kEpsilon)) {
            continue;
        }

        double vx = eigenvectors[0][order[0]];
        double vy = eigenvectors[1][order[0]];
        double vz = eigenvectors[2][order[0]];
        const double norm = std::sqrt(vx * vx + vy * vy + vz * vz);
        if (!(norm > kEpsilon)) {
            continue;
        }

        vx /= norm;
        vy /= norm;
        vz /= norm;

        out_principal_dirs[query_idx * 3 + 0] = vx;
        out_principal_dirs[query_idx * 3 + 1] = vy;
        out_principal_dirs[query_idx * 3 + 2] = vz;
        out_linearness[query_idx] = (lambda1 - lambda2) / lambda1;
        out_valid[query_idx] = 1;
    }

    return 0;
}
