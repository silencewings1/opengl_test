#pragma once
#include <vector>
#include <Eigen/Core>

struct PlyColor
{
    int r, g, b;
};

struct PlyVertex
{
    Eigen::Vector3d pos;
    PlyColor color;
};

using PlyVertexes = std::vector<PlyVertex>;