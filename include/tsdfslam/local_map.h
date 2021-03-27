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
class LocalMap
{
public:
    LocalMap() : _center_x{SX < 2 ? 1 : SX/2}, _center_y{SY < 2 ? 1 : SY/2}, _center_z{SZ < 2 ? 1 : SZ/2}
    {
        static_assert(SX > 2 && SX % 2 == 1, "Axis must be at least 3 && ungerade");
        static_assert(SY > 2 && SY % 2 == 1, "Axis must be at least 3 && ungerade");
        static_assert(SZ > 2 && SZ % 2 == 1, "Axis must be at least 3 && ungerade");
    }

    ~LocalMap() = default;

    void update(const Eigen::Vector3i& center, DTYPE value)
    {
		auto index = _3d21d(center);
		if (index == -1)
		{
			return;
		}
		
		if (_tsdf_values.at(index) > value && _tsdf_values.at(index) != 0)
		{
			_tsdf_values[index] = value;
		}
    }
	
    template <typename T>
    Eigen::Vector3i xyz2ijk(const T& center) const
    {
        return {center[2], center[1], center[0]};
    }

    Eigen::Vector3i shift_to_local_map(const Eigen::Vector3i& center) const
    {
        return { _center_x + center[0], _center_y + center[1], _center_z + center[2] };
    }
    
	int _3d21d(const Eigen::Vector3i& center)
	{
    	// bounds check
    	if (center[0] > SX / 2 || center[1] > SY / 2 || center[2] > SZ / 2)
		{
    		return -1;
		}
    	
    	// shift into local coordinate system
		auto center_shifted = shift_to_local_map(center);
    	
    	// convert xyz representation into ijk
    	// x -> depth, y -> width, z -> height
		auto center_axis_shifted = xyz2ijk(center_shifted);
		
		int i = center_axis_shifted[0];
		int j = center_axis_shifted[1];
		int k = center_axis_shifted[2];
		
		return i * SY * SZ + j * SZ + k;
	}

    size_t size()
    {
        return _size;
    }

    const std::array<DTYPE, SX*SY*SZ> map() const
    {
        return _tsdf_values;
    }
    
private:
    constexpr static size_t _size = SX * SY * SZ;
    constexpr static std::array<DTYPE, SX * SY * SZ> _tsdf_values = {};
    int _center_x;
    int _center_y;
    int _center_z;
};

bool operator==(const Eigen::Vector3i& center, const std::tuple<size_t, size_t, size_t>& tuple)
{
    auto [i, j, k] = tuple;
    return center[0] == i && center[1] == j && center[2] == k;
}

}