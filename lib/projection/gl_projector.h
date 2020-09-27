#pragma once
#include <Eigen/Core>
#include <GL/glut.h>

class GLProjector
{
public:
    GLProjector()
    {
        glGetDoublev(GL_MODELVIEW_MATRIX, modelView);
        glGetDoublev(GL_PROJECTION_MATRIX, projection);
        glGetIntegerv(GL_VIEWPORT, viewport);
    }

    // Project:    object->screen
    // UnProject:  screen->object
    Eigen::Vector3d UnProject(double inX, double inY, double inZ) const
    {
        double x, y, z;
        gluUnProject(inX, inY, inZ, modelView, projection, viewport, &x, &y, &z);
        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector3d UnProject(const Eigen::Vector3d& p) const
    {
        return UnProject(p(0), p(1), p(2));
    }

    Eigen::Vector3d Project(double inX, double inY, double inZ) const
    {
        double x, y, z;
        gluProject(inX, inY, inZ, modelView, projection, viewport, &x, &y, &z);
        return Eigen::Vector3d(x, y, z);
    }

    Eigen::Vector3d Project(const Eigen::Vector3d& p) const
    {
        return Project(p(0), p(1), p(2));
    }

private:
    double modelView[16];
    double projection[16];
    int viewport[4];
};