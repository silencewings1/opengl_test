#pragma once
#include <vector>

struct PlyPosition
{
    double x, y, z;
};

struct PlyColor
{
    int r, g, b;
};

struct PlyVertex
{
    PlyPosition pos;
    PlyColor color;
};

using PlyVertexes = std::vector<PlyVertex>;