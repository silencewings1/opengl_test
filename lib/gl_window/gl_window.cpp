#include "gl_window.h"
#include "fnptr.h"
#include "math/quadric_surface.h"
#include "math/tk_spline.h"
#include <algorithm>
#include <iostream>
#include <numeric>

namespace
{

using VertexSet = std::vector<Eigen::Vector3d>;

VertexSet PreProcessCurve(const VertexSet& curve_vertex)
{
    if (curve_vertex.size() <= 3)
    {
        printf("size of curve_vertex no more than 3\n");
        return curve_vertex;
    }

    auto&& ver = curve_vertex;

    auto distance = [](const Eigen::Vector3d& lhs, const Eigen::Vector3d& rhs) {
        return (lhs - rhs).norm();
    };
    auto distance_average = [&distance](const VertexSet& set) {
        std::vector<double> dis_set(set.size() - 1);
        for (size_t i = 0; i < set.size() - 1; ++i)
        {
            dis_set[i] = distance(set[i + 1], set[i]);
        }
        return std::accumulate(dis_set.begin(), dis_set.end(), 0.0) / dis_set.size();
    };

    // remove points too far
    VertexSet ver2;
    {
        const double avg_dis = distance_average(ver);
        const double dis_max = 3 * avg_dis;
        if (distance(ver[0], ver[1]) < dis_max ||
            distance(ver[0], ver[2]) < dis_max)
        {
            ver2.push_back(ver.front());
        }
        for (size_t i = 1; i < ver.size() - 1; ++i)
        {
            auto dis_left = distance(ver[i - 1], ver[i]);
            auto dis_right = distance(ver[i], ver[i + 1]);
            if (dis_left < dis_max &&
                dis_right < dis_max)
            {
                ver2.push_back(ver[i]);
            }
        }
        const auto eid = ver.size() - 1;
        if (distance(ver[eid], ver[eid - 1]) < dis_max ||
            distance(ver[eid], ver[eid - 2]) < dis_max)
        {
            ver2.push_back(ver.back());
        }

        if (ver2.size() < 3)
        {
            printf("size of ver2 less than 3\n");
            return ver;
        }
    }

    // remove points too close
    VertexSet new_ver = {ver2.front()};
    {
        const double avg_dis = distance_average(ver2);
        for (size_t i = 0; i < ver2.size() - 1;)
        {
            size_t j = i + 1;
            for (; j < ver2.size(); ++j)
            {
                if (distance(ver2[i], ver2[j]) > avg_dis * 0.5)
                {
                    new_ver.push_back(ver2[j]);
                    i = j;
                    break;
                }
            }
            if (j == ver2.size())
                break;
        }

        if (new_ver.size() < 3)
        {
            printf("size of new_ver less than 3\n");
            return ver2;
        }
    }

    return new_ver;
}

VertexSet CreateSpline(const VertexSet& curve_vertex)
{
    if (curve_vertex.size() <= 3)
    {
        printf("size of curve_vertex no more than 3\n");
        return curve_vertex;
    }

    const auto size = curve_vertex.size();
    std::vector<double> X(size), Y(size), Z(size), t(size);

    for (size_t i = 0; i < size; ++i)
    {
        X[i] = curve_vertex[i].x();
        Y[i] = curve_vertex[i].y();
        Z[i] = curve_vertex[i].z();
        t[i] = static_cast<double>(i);
    }

    tk::spline sx, sy, sz;
    sx.set_points(t, X);
    sy.set_points(t, Y);
    sz.set_points(t, Z);

    VertexSet res;
    constexpr auto step = 0.1;
    for (double id = 0; id <= static_cast<double>(size - 1) + step; id += step)
    {
        res.emplace_back(sx(id), sy(id), sz(id));
    }

    return res;
}

void ProjectCurveOntoSurface(const VertexSet& surface_vertex,
                             VertexSet& curve_vertex)
{
    auto surface = QuadricSurfaceSolver::Solve(surface_vertex);
    if (!surface.Valid())
    {
        return;
    }

    for (auto& vertex : curve_vertex)
    {
        auto dis = surface.Value(vertex);
        while (abs(dis) > 1e-7)
        {
            auto& val = dis;
            auto grad = surface.Gradiant(vertex);
            vertex += -val * grad / (pow(grad.norm(), 2));

            dis = surface.Value(vertex);
        }
    }
}

} // namespace

GlWindow::GlWindow(const std::string& window_name)
    : g_fov{45.0}
    , leftDown(false)
{
    InitGL(window_name);
    InitMenu();
    SetCallbacks();
}

void GlWindow::InitGL(const std::string& window_name)
{
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow(window_name.c_str());

    glClearColor(0.0, 0.0, 0.0, 0.0);
    glPolygonOffset(1.0, 1.0);
    glDepthFunc(GL_LEQUAL);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_CULL_FACE);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT, GL_DIFFUSE);

    GLfloat light0_position[] = {0, 1, 0, 1.0};
    glLightfv(GL_LIGHT0, GL_POSITION, light0_position);
    glEnable(GL_LIGHT0);
}

void GlWindow::InitMenu()
{
    glutCreateMenu(FnPtr<void(int)>([this](int value) {
        switch (value)
        {
        case 101:
            curve_vertex = PreProcessCurve(curve_vertex);
            spline_vertex = CreateSpline(curve_vertex);
            break;
        case 102:
            spline_vertex.clear();
            break;
        case 103:
            ProjectCurveOntoSurface(rect_box_vertex, curve_vertex);
            spline_vertex = CreateSpline(curve_vertex);
            break;
        case 27:
            exit(0);
            break;
        }
    }));

    glutAddMenuEntry("spline", 101);
    glutAddMenuEntry("reset", 102);
    glutAddMenuEntry("surface", 103);
    glutAddMenuEntry("exit", 27);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void GlWindow::SetCallbacks()
{
    glutReshapeFunc(FnPtr<void(int, int)>(
        [this](int w, int h) { ReshapeFunc(w, h); }));

    glutDisplayFunc(FnPtr<void()>(
        [this]() { DisplayFunc(); }));

    glutKeyboardFunc(FnPtr<void(unsigned char, int, int)>(
        [this](unsigned char ch, int x, int y) { KeyboardFunc(ch, x, y); }));

    glutKeyboardUpFunc(FnPtr<void(unsigned char, int, int)>(
        [this](unsigned char ch, int x, int y) { KeyboardUpFunc(ch, x, y); }));

    glutMouseFunc(FnPtr<void(int, int, int, int)>(
        [this](int btn, int state, int x, int y) { MouseFunc(btn, state, x, y); }));

    glutMotionFunc(FnPtr<void(int, int)>(
        [this](int x, int y) { MotionFunc(x, y); }));
}

void GlWindow::ReshapeFunc(int width, int height)
{
    win_width = width;
    win_height = height;
    win_aspect = (double)width / (double)height;
    glViewport(0, 0, width, height);
    glutPostRedisplay();
}

void GlWindow::DisplayFunc()
{
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(g_fov, win_aspect, zNear, zFar);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glTranslatef(xpan, ypan, -sdepth);
    glRotatef(-stheta, 1.0, 0.0, 0.0);
    glRotatef(sphi, 0.0, 1.0, 0.0);
    glTranslatef(-g_center[0], -g_center[1], -g_center[2]);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (draw_frame_func)
        draw_frame_func();

    DrawRectBoxVertex();
    DrawCurveVertex();
    DrawSpline();

    glutSwapBuffers();
}

void GlWindow::DrawRectBoxVertex()
{
    glPointSize(5.0);
    glColor3f(1.0, 0.5, 0.25);
    glBegin(GL_POINTS);
    for (const auto& vertex : rect_box_vertex)
    {
        glVertex3d(vertex.x(), vertex.y(), vertex.z());
    }
    glEnd();
}

void GlWindow::DrawCurveVertex()
{
    glPointSize(5.0);
    glColor3f(0.2, 1.0, 0.2);
    glBegin(GL_POINTS);
    for (const auto& vertex : curve_vertex)
    {
        glVertex3d(vertex.x(), vertex.y(), vertex.z());
    }
    glEnd();
}

void GlWindow::DrawSpline()
{
    glLineWidth(5);
    glColor3f(1.0, 0.2, 0.2);
    glBegin(GL_LINE_STRIP);
    for (const auto& vertex : spline_vertex)
    {
        glVertex3d(vertex.x(), vertex.y(), vertex.z());
    }
    glEnd();
}

void GlWindow::KeyboardFunc(unsigned char ch, int x, int y)
{
    switch (ch)
    {
    case 27: // Esc
        exit(0);
        break;
    case 'r':
    case 'R':
        rect_down = true;
        break;
    case 'c':
    case 'C':
        curve_down = true;
        break;
    }

    glutPostRedisplay();
}

void GlWindow::KeyboardUpFunc(unsigned char ch, int x, int y)
{
    switch (ch)
    {
    case 'r':
    case 'R':
        rect_down = false;
        break;
    case 'c':
    case 'C':
        curve_down = false;
        break;
    }

    glutPostRedisplay();
}

void GlWindow::MouseFunc(int button, int state, int x, int y)
{
    leftDown = (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN);
    middleDown = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN);
    shiftDown = (glutGetModifiers() & GLUT_ACTIVE_SHIFT);
    bool rightDown = (button == GLUT_RIGHT_BUTTON) && (state == GLUT_DOWN);

    if (rightDown)
    {
        rect_box_vertex.clear();
        curve_vertex.clear();
    }

    if (rect_down && leftDown)
    {
        rect_x = x;
        rect_y = y;
        rect_box_vertex.clear();
    }

    if (curve_down && leftDown)
    {
        curve_vertex.clear();
    }

    lastX = x;
    lastY = y;
    glutPostRedisplay();
}

void GlWindow::MotionFunc(int x, int y)
{
    if (leftDown && rect_down) // for draw rect
    {
        if (rect_box_func)
        {
            rect_box_vertex = rect_box_func(
                rect_x, win_height - rect_y, x, win_height - y);
        }
    }
    else if (leftDown && curve_down)
    {
        if (curve_func)
        {
            Eigen::Vector3d ver = curve_func(x, win_height - y);
            if (!ver.isZero())
            {
                if (curve_vertex.empty() ||
                    (ver - curve_vertex.back()).norm() > 1e-7)
                {
                    curve_vertex.push_back(ver);
                }
            }
        }
    }
    else if (leftDown && shiftDown) // pan with shift key
    {
        xpan += (double)(x - lastX) * sdepth / zNear / win_width;
        ypan += (double)(lastY - y) * sdepth / zNear / win_height;
    }
    else if (leftDown && !shiftDown) // rotate
    {
        sphi += (double)(x - lastX) / 4.0;
        stheta += (double)(lastY - y) / 4.0;
    }
    else if (middleDown) // scale
    {
        sdepth += (double)(lastY - y) / 10.0;
    }

    lastX = x;
    lastY = y;
    glutPostRedisplay();
}

void GlWindow::SetBoundaryBox(const WinBoundary& bound)
{
    auto& bmin = bound.wmin;
    auto& bmax = bound.wmax;

    double PI = 3.14159265358979323846;
    double radius = (bmax - bmin).norm();
    g_center = 0.5 * (bmin + bmax);
    zNear = 0.2 * radius / sin(0.5 * g_fov * PI / 180.0);
    zFar = zNear + 2.0 * radius;
    zNear *= 0.1;
    zFar *= 10;
    sdepth = zNear + radius;
}