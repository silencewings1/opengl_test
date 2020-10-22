#pragma once
#include <string>

class GlobalSfM
{
public:
    GlobalSfM(const std::string images_dir,
              const std::string output_dir);

    bool Solve() const;
    std::string GetModelPath() const;

private:
    const std::string images_dir;
    const std::string output_dir;
    const std::string maps_dir = "/home/ospacer/Documents/resource/map";
};
