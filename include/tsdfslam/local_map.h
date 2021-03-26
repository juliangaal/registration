#pragma once

/**
  * @file local_map.h
  * @author julian gaal
  * @date 03/22/21
 */

#include <Eigen/Dense>
#include <tuple>
#include <array>

namespace tsdfslam
{

template <size_t SX, size_t SY, size_t SZ, typename DTYPE = float>
struct LocalMap
{
    LocalMap() : _center_x{SX < 2 ? 1 : SX/2}, _center_y{SY < 2 ? 1 : SY/2}, _center_z{SZ < 2 ? 1 : SZ/2}
    {
        static_assert(SX > 2 && SX % 2 == 1, "Axis must be at least 3 && ungerade");
        static_assert(SY > 2 && SY % 2 == 1, "Axis must be at least 3 && ungerade");
        static_assert(SZ > 2 && SZ % 2 == 1, "Axis must be at least 3 && ungerade");
    }

    ~LocalMap() = default;

    void update(const Eigen::Vector3i& center, DTYPE value)
    {

    }

    std::tuple<size_t, size_t, size_t> xyz2ijk(const Eigen::Vector3i& center) const
    {
        return {center[2], center[1], center[0]};
    }

    std::tuple<size_t, size_t, size_t> shift_to_local_map(const Eigen::Vector3i& center) const
    {
        return { _center_x + center[0], _center_y + center[1], _center_z + center[2] };
    }

    size_t size()
    {
        return _size;
    }

    const std::array<DTYPE, SX*SY*SZ> map() const
    {
        return _tsdf_values;
    }

    constexpr static size_t _size = SX * SY * SZ;
    constexpr static std::array<DTYPE, SX * SY * SZ> _tsdf_values = {};
    size_t _center_x;
    size_t _center_y;
    size_t _center_z;
};

bool operator==(const Eigen::Vector3i& center, const std::tuple<size_t, size_t, size_t>& tuple)
{
    auto [i, j, k] = tuple;
    return center[0] == i && center[1] == j && center[2] == k;
}

}