#include "disparity_display.h"
#include <iostream>

int main(int argc, char** argv)
{
    glutInit(&argc, argv);

    auto sgbm_solver = SgbmSolver("/home/ospacer/Documents/resource",
                                  "/map",
                                  "/images/heart_model2/left",
                                  "/images/heart_model2/right");
    WinBoundary win_bound;
    auto model = sgbm_solver.Solve("left_3.png", "right_3.png", win_bound);
    std::cout << "model vertex count:" << model.size() << std::endl;

    auto viewer = GlWindow("display");
    viewer.SetBoundaryBox(win_bound);
    viewer.SetDrawFrameFunc([&model]() {
        glPointSize(2.0);
        glBegin(GL_POINTS);
        for (const auto& vertex : model)
        {
            glColor3d(vertex.color.r / 255.0, vertex.color.g / 255.0, vertex.color.b / 255.0);
            glVertex3d(vertex.pos.x(), vertex.pos.y(), vertex.pos.z());
        }
        glEnd();
    });

    viewer.SetRectBoxFunc([&model](int x1, int y1, int x2, int y2) {
        std::vector<Eigen::Vector3d> selected;

        GLProjector projector;
        for (const auto& vertex : model)
        {
            Eigen::Vector3d pixel_pos = projector.Project(vertex.pos);
            if (pixel_pos.x() > std::min(x1, x2) &&
                pixel_pos.x() < std::max(x1, x2) &&
                pixel_pos.y() > std::min(y1, y2) &&
                pixel_pos.y() < std::max(y1, y2))
            {
                selected.push_back(vertex.pos);
            }
        }

        return selected;
    });

    viewer.SetCurveFunc([&model](int x, int y) {
        Eigen::Vector3d res = Eigen::Vector3d::Zero();
        double dis_min = 1e10;

        GLProjector projector;
        for (const auto& vertex : model)
        {
            Eigen::Vector3d pixel_pos = projector.Project(vertex.pos);
            double dis = sqrt(pow((pixel_pos.x() - x), 2) +
                              pow((pixel_pos.y() - y), 2));
            if (dis < 3.0)
            {
                dis_min = std::min(dis_min, dis);
                res = vertex.pos;
            }
        }

        return res;
    });

    glutMainLoop();
    return 0;
}