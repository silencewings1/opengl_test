#pragma once
#include <vector>
#include <Eigen/Core>

struct ModelColor
{
    int r, g, b;
};

struct ModelVertex
{
    Eigen::Vector3d pos;
    ModelColor color;
};

using Model = std::vector<ModelVertex>;