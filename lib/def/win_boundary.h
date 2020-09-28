#pragma once
#include <Eigen/Core>

struct WinBoundary
{
    WinBoundary()
        : wmin(1e10 * Eigen::Vector3d::Ones())
        , wmax(-1e10 * Eigen::Vector3d::Ones())
    {
    }

    Eigen::Vector3d wmin;
    Eigen::Vector3d wmax;
};
