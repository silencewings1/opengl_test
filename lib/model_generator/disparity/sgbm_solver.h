#include "def/model.h"
#include "def/win_boundary.h"
#include "rectify/rectifier.h"
#include <string>

class SgbmSolver
{
public:
    SgbmSolver(const std::string& resource_path,
               const std::string& map_floder,
               const std::string& left_image_folder,
               const std::string& right_image_folder);

    // from left image
    Model Solve(const std::string& left_image_name,
                const std::string& right_image_name,
                WinBoundary& bound) const;

private:
    const std::string resource_path;
    const std::string left_image_path;
    const std::string right_image_path;

    Rectifier rectifier;
};