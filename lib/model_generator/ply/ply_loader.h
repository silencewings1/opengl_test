#pragma once
#include "def/model.h"
#include "def/win_boundary.h"
#include <string>

class PlyLoader
{
public:
    PlyLoader(const std::string& ply_path);
    Model Load(WinBoundary& bound) const;

private:
    const std::string ply_path;
};