#pragma once
#include <Eigen/Core>
#include <GL/glut.h>
#include <functional>
#include <string>
#include <vector>

class GlWindow
{
public:
    using DrawFrameFunc = std::function<void()>;
    using RectBoxFunc = std::function<std::vector<Eigen::Vector3d>(int, int, int, int)>;
    using CurveFunc = std::function<Eigen::Vector3d(int, int)>;

public:
    GlWindow(const std::string& window_name);

    void SetBoundaryBox(const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax);

    void SetDrawFrameFunc(const DrawFrameFunc& func) { draw_frame_func = func; }
    void SetRectBoxFunc(const RectBoxFunc& func) { rect_box_func = func; }
    void SetCurveFunc(const CurveFunc& func) { curve_func = func; }

private:
    void Init(const std::string& window_name);
    void SetCallbacks();

    // cb
    void ReshapeFunc(int width, int height);
    void DisplayFunc();
    void KeyboardFunc(unsigned char ch, int x, int y);
    void KeyboardUpFunc(unsigned char ch, int x, int y);
    void MouseFunc(int button, int state, int x, int y);
    void MotionFunc(int x, int y);

    void DrawRectBoxVertex();
    void DrawCurveVertex();

private:
    DrawFrameFunc draw_frame_func;

    RectBoxFunc rect_box_func;
    std::vector<Eigen::Vector3d> rect_box_vertex;

    CurveFunc curve_func;
    std::vector<Eigen::Vector3d> curve_vertex;

private:
    const double g_fov;

    int win_width, win_height;
    double win_aspect;

    double zNear = 1.0, zFar = 100.0;

    double sphi = 90.0, stheta = 45.0, sdepth = 10;
    double xpan = 0.0, ypan = 0.0;

    Eigen::Vector3d g_center;

    // mouse
    bool leftDown, middleDown, shiftDown; // mouse/shift down flags
    int lastX, lastY;                     // last mouse motion position
    int downX, downY;                     // mouse down position

    // rect draw box
    bool rect_down = false;
    int rect_x, rect_y;

    // curve
    bool curve_down = false;
};