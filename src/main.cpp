#include "ply_viewer/ply_viewer.h"

int main(int argc, char** argv)
{
    glutInit(&argc, argv);

    auto loader = PlyLoader("model_ply/cube.ply");
    auto model = loader.Load();

    auto viewer = GlWindow("display");
    viewer.SetBoundaryBox(loader.MinBound(), loader.MaxBound());
    viewer.SetDrawFrameFunc([&model]() {
        glPointSize(2.0);
        glBegin(GL_POINTS);
        for (const auto& vertex : model)
        {
            glColor3d(vertex.color.r / 255.0, vertex.color.g / 255.0, vertex.color.b / 255.0);
            glVertex3d(vertex.pos.x, vertex.pos.y, vertex.pos.z);
        }
        glEnd();
    });

    glutMainLoop();
    return 0;
}