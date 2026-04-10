#include <cmath>
#include <cstdint>
#include <functional>
#include <unordered_map>

#if defined(_WIN32)
#define FILTER_EXPORT extern "C" __declspec(dllexport)
#else
#define FILTER_EXPORT extern "C"
#endif

namespace {

struct VoxelKey {
    int64_t x;
    int64_t y;
    int64_t z;

    bool operator==(const VoxelKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey& key) const noexcept {
        std::size_t h1 = std::hash<int64_t>{}(key.x);
        std::size_t h2 = std::hash<int64_t>{}(key.y);
        std::size_t h3 = std::hash<int64_t>{}(key.z);
        return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
};

}  // namespace

FILTER_EXPORT int64_t voxel_downsample_indices(
    const double* points,
    int64_t n_points,
    double voxel_size,
    int64_t* out_indices,
    int64_t out_capacity) {
    if (points == nullptr || out_indices == nullptr || n_points < 0 || voxel_size <= 0.0) {
        return -1;
    }

    std::unordered_map<VoxelKey, int64_t, VoxelKeyHash> first_by_voxel;
    first_by_voxel.reserve(static_cast<std::size_t>(n_points));

    int64_t out_count = 0;
    for (int64_t idx = 0; idx < n_points; ++idx) {
        const double x = points[idx * 3 + 0];
        const double y = points[idx * 3 + 1];
        const double z = points[idx * 3 + 2];
        const VoxelKey key{
            static_cast<int64_t>(std::floor(x / voxel_size)),
            static_cast<int64_t>(std::floor(y / voxel_size)),
            static_cast<int64_t>(std::floor(z / voxel_size)),
        };

        if (first_by_voxel.find(key) != first_by_voxel.end()) {
            continue;
        }
        first_by_voxel.emplace(key, idx);
        if (out_count < out_capacity) {
            out_indices[out_count] = idx;
        }
        ++out_count;
    }

    return out_count;
}

FILTER_EXPORT void crop_mask(
    const double* points,
    int64_t n_points,
    double xmin,
    double xmax,
    double ymin,
    double ymax,
    double zmin,
    double zmax,
    uint8_t* out_mask) {
    if (points == nullptr || out_mask == nullptr || n_points < 0) {
        return;
    }

    for (int64_t idx = 0; idx < n_points; ++idx) {
        const double x = points[idx * 3 + 0];
        const double y = points[idx * 3 + 1];
        const double z = points[idx * 3 + 2];
        out_mask[idx] = (x >= xmin && x <= xmax && y >= ymin && y <= ymax && z >= zmin && z <= zmax) ? 1 : 0;
    }
}
