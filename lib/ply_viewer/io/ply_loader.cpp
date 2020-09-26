#include "ply_loader.h"
#include <fstream>
#include <sstream>

PlyLoader::PlyLoader(const std::string& ply_path)
    : ply_path{ply_path}
    , min_bound{Eigen::Vector3d::Zero()}
    , max_bound{Eigen::Vector3d::Zero()}
{
}

PlyVertexes PlyLoader::Load()
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

    min_bound = 10000 * Eigen::Vector3d::Ones();
    max_bound = -10000 * Eigen::Vector3d::Ones();
    PlyVertexes ver;
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

        min_bound(0) = std::min(min_bound(0), x);
        min_bound(1) = std::min(min_bound(1), y);
        min_bound(2) = std::min(min_bound(2), z);
        max_bound(0) = std::max(max_bound(0), x);
        max_bound(1) = std::max(max_bound(1), y);
        max_bound(2) = std::max(max_bound(2), z);

        getline(ss, str, ' ');
        auto r = std::stoi(str);
        getline(ss, str, ' ');
        auto g = std::stoi(str);
        getline(ss, str, ' ');
        auto b = std::stoi(str);

        ver.push_back({{x, y, z}, {r, g, b}});
    }

    return ver;
}