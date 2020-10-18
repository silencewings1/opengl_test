#include "global_sfm.h"

int main()
{
    const std::string images_dir = "/home/ospacer/Documents/3d/project/file/image7/images/";
    const std::string output_dir = "../output/";
    auto global_sfm_solver = GlobalSfM(images_dir, output_dir);

    global_sfm_solver.Solve();

    return 0;
}