#pragma once
#include "def/ply_def.h"
#include <Eigen/Core>
#include <string>

class PlyLoader
{
public:
    PlyLoader(const std::string& ply_path);

    PlyVertexes Load();
    const Eigen::Vector3d& MinBound() const { return min_bound; }
    const Eigen::Vector3d& MaxBound() const { return max_bound; }

private:
    const std::string ply_path;
    Eigen::Vector3d min_bound;
    Eigen::Vector3d max_bound;
};