#pragma once

/**
  * @file local_map.h
  * @author julian gaal
  * @date 03/22/21
 */

#include <Eigen/Dense>
#include <array>

namespace tsdfslam
{

template <size_t SX, size_t SY, size_t SZ, typename DTYPE = float>
struct LocalMap
{
    LocalMap() : _tsdf_values{} {}

    ~LocalMap() = default;

    void update(const Eigen::Vector3i& center, DTYPE value)
    {

    }

    size_t size()
    {
        return _size;
    }

    constexpr static size_t _size = SX * SY * SZ;
    std::array<DTYPE, SX * SY * SZ> _tsdf_values;
    constexpr static size_t _center_x = SX / 2;
    constexpr static size_t _center_y = SY / 2;
    constexpr static size_t _center_z = SZ / 2;
};

}