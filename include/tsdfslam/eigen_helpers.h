#pragma once

/**
  * @file eigen_helpers.h
  * @author julian gaal
  * @date 03/22/21
 */

#include <Eigen/Dense>

Eigen::Vector3f operator-(const Eigen::Vector3f& a, const Eigen::Vector3i& b)
{
    return {a[0] - static_cast<float>(b[0]), a[1] - static_cast<float>(b[1]), a[2] - static_cast<float>(b[2])};
}