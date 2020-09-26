#include "gl_window.h"
#include "fnptr.h"

GlWindow::GlWindow(const std::string& window_name)
    : g_fov{45.0}
{
    Init(window_name);
    SetCallbacks();
}

void GlWindow::Init(const std::string& window_name)
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

void GlWindow::SetCallbacks()
{
    glutReshapeFunc(FnPtr<void(int, int)>(
        [this](int w, int h) { ReshapeFunc(w, h); }));

    glutDisplayFunc(FnPtr<void()>(
        [this]() { DisplayFunc(); }));

    glutKeyboardFunc(FnPtr<void(unsigned char, int, int)>(
        [this](unsigned char ch, int x, int y) { KeyboardFunc(ch, x, y); }));

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

    glutSwapBuffers();
}

void GlWindow::KeyboardFunc(unsigned char ch, int x, int y)
{
    switch (ch)
    {
    case 27: // Esc
        exit(0);
        break;
    }

    glutPostRedisplay();
}

void GlWindow::MouseFunc(int button, int state, int x, int y)
{
    if (state == GLUT_DOWN)
    {
        downX = x;
        downY = y;
    }
    lastX = x;
    lastY = y;
    leftDown = (button == GLUT_LEFT_BUTTON) && (state == GLUT_DOWN);
    middleDown = (button == GLUT_MIDDLE_BUTTON) && (state == GLUT_DOWN);
    shiftDown = (glutGetModifiers() & GLUT_ACTIVE_SHIFT);
}

void GlWindow::MotionFunc(int x, int y)
{
    if (leftDown)
    {
        if (!shiftDown)
        { // rotate
            sphi += (double)(x - lastX) / 4.0;
            stheta += (double)(lastY - y) / 4.0;
        }
        else
        { // pan with shift key
            xpan += (double)(x - lastX) * sdepth / zNear / win_width;
            ypan += (double)(lastY - y) * sdepth / zNear / win_height;
        }
    }
    // scale
    if (middleDown)
    {
        sdepth += (double)(lastY - y) / 10.0;
    }

    lastX = x;
    lastY = y;

    glutPostRedisplay();
}

void GlWindow::SetBoundaryBox(const Eigen::Vector3d& bmin, const Eigen::Vector3d& bmax)
{
    double PI = 3.14159265358979323846;
    double radius = (bmax - bmin).norm();
    g_center = 0.5 * (bmin + bmax);
    zNear = 0.2 * radius / sin(0.5 * g_fov * PI / 180.0);
    zFar = zNear + 2.0 * radius;
    zNear *= 0.1;
    zFar *= 10;
    sdepth = zNear + radius;
}