#pragma once
#include <Eigen/Core>
#include <GL/glut.h>
#include <functional>
#include <string>

class GlWindow
{
public:
    using DrawFrameFunc = std::function<void()>;

public:
    GlWindow(const std::string& window_name);

    void SetDrawFrameFunc(const DrawFrameFunc& func) { draw_frame_func = func; }
    void SetBoundaryBox(const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax);

private:
    void Init(const std::string& window_name);
    void SetCallbacks();

    // cb
    void ReshapeFunc(int width, int height);
    void DisplayFunc();
    void KeyboardFunc(unsigned char ch, int x, int y);
    void MouseFunc(int button, int state, int x, int y);
    void MotionFunc(int x, int y);

private:
    DrawFrameFunc draw_frame_func;

private:
    const double g_fov;

private:
    int win_width, win_height;
    double win_aspect;

    double zNear = 1.0, zFar = 100.0;

    double sphi = 90.0, stheta = 45.0, sdepth = 10;
    double xpan = 0.0, ypan = 0.0;

    Eigen::Vector3d g_center;

    // mouse
    bool leftDown, middleDown, shiftDown; // mouse down and shift down flags
    int lastX, lastY;                     // last mouse motion position
    int downX, downY;                     // mouse down position
};