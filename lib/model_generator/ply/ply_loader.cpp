#include "ply_loader.h"
#include <fstream>
#include <sstream>

PlyLoader::PlyLoader(const std::string& ply_path)
    : ply_path{ply_path}
{
}

Model PlyLoader::Load(WinBoundary& bound) const
{
    std::ifstream file(ply_path, std::ios::in);
    std::string line;

    // check ply
    getline(file, line);
    if (line != "ply")
    {
        return {};
    }

    // vertex counts
    int loop_count = 20;
    int vertex_count = 0;
    while (getline(file, line) && loop_count-- > 0)
    {
        std::stringstream ss(line);
        std::string str;
        getline(ss, str, ' ');
        if (str == "element")
        {
            getline(ss, str, ' ');
            if (str == "vertex")
            {
                getline(ss, str, ' ');
                vertex_count = std::stoi(str);
                break;
            }
        }
    }
    if (loop_count == 0)
    {
        return {};
    }

    // end_header
    loop_count = 20;
    while (getline(file, line) && loop_count-- > 0)
    {
        if (line == "end_header")
        {
            break;
        }
    }
    if (loop_count == 0)
    {
        return {};
    }

    Model model;
    for (int i = 0; i < vertex_count; ++i)
    {
        getline(file, line);

        std::stringstream ss(line);
        std::string str;

        getline(ss, str, ' ');
        auto x = std::stod(str);
        getline(ss, str, ' ');
        auto y = std::stod(str);
        getline(ss, str, ' ');
        auto z = std::stod(str);

        getline(ss, str, ' ');
        auto r = std::stoi(str);
        getline(ss, str, ' ');
        auto g = std::stoi(str);
        getline(ss, str, ' ');
        auto b = std::stoi(str);

        model.push_back({Eigen::Vector3d(x, y, z), {r, g, b}});

        bound.wmin(0) = std::min(bound.wmin(0), x);
        bound.wmin(1) = std::min(bound.wmin(1), y);
        bound.wmin(2) = std::min(bound.wmin(2), z);
        bound.wmax(0) = std::max(bound.wmax(0), x);
        bound.wmax(1) = std::max(bound.wmax(1), y);
        bound.wmax(2) = std::max(bound.wmax(2), z);
    }

    return model;
}