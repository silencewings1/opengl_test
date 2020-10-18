#include "sgbm_solver.h"
#include <opencv2/opencv.hpp>

#define USE_STEREO 1
#include "def/cam_para.h"
using namespace CameraPara;

namespace
{

void FillDepthMap32F(cv::Mat& depth)
{
    const int width = depth.cols;
    const int height = depth.rows;
    float* data = (float*)depth.data;
    cv::Mat integralMap = cv::Mat::zeros(height, width, CV_64F);
    cv::Mat ptsMap = cv::Mat::zeros(height, width, CV_32S);
    double* integral = (double*)integralMap.data;
    int* ptsIntegral = (int*)ptsMap.data;
    memset(integral, 0, sizeof(double) * width * height);
    memset(ptsIntegral, 0, sizeof(int) * width * height);
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            if (data[id2] > 1e-3)
            {
                integral[id2] = data[id2];
                ptsIntegral[id2] = 1;
            }
        }
    }
    // 积分区间
    for (int i = 0; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 1; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - 1];
            ptsIntegral[id2] += ptsIntegral[id2 - 1];
        }
    }
    for (int i = 1; i < height; ++i)
    {
        int id1 = i * width;
        for (int j = 0; j < width; ++j)
        {
            int id2 = id1 + j;
            integral[id2] += integral[id2 - width];
            ptsIntegral[id2] += ptsIntegral[id2 - width];
        }
    }
    int wnd;
    double dWnd = 2;
    while (dWnd > 1)
    {
        wnd = int(dWnd);
        dWnd /= 2;
        for (int i = 0; i < height; ++i)
        {
            int id1 = i * width;
            for (int j = 0; j < width; ++j)
            {
                int id2 = id1 + j;
                int left = j - wnd - 1;
                int right = j + wnd;
                int top = i - wnd - 1;
                int bot = i + wnd;
                left = std::max(0, left);
                right = std::min(right, width - 1);
                top = std::max(0, top);
                bot = std::min(bot, height - 1);
                int dx = right - left;
                int dy = (bot - top) * width;
                int idLeftTop = top * width + left;
                int idRightTop = idLeftTop + dx;
                int idLeftBot = idLeftTop + dy;
                int idRightBot = idLeftBot + dx;
                int ptsCnt = ptsIntegral[idRightBot] + ptsIntegral[idLeftTop] - (ptsIntegral[idLeftBot] + ptsIntegral[idRightTop]);
                double sumGray = integral[idRightBot] + integral[idLeftTop] - (integral[idLeftBot] + integral[idRightTop]);
                if (ptsCnt <= 0)
                {
                    continue;
                }
                data[id2] = float(sumGray / ptsCnt);
            }
        }
        int s = wnd / 2 * 2 + 1;
        if (s > 201)
        {
            s = 201;
        }
        cv::GaussianBlur(depth, depth, cv::Size(s, s), s, s);
    }
}

} // namespace

SgbmSolver::SgbmSolver(const std::string& resource_path,
                       const std::string& map_floder,
                       const std::string& left_image_folder,
                       const std::string& right_image_folder)
    : resource_path(resource_path)
    , left_image_path(resource_path + left_image_folder)
    , right_image_path(resource_path + right_image_folder)
    , rectifier(cv::Size(1920, 1080), resource_path + map_floder)
{
}

Model SgbmSolver::Solve(const std::string& left_image_name,
                        const std::string& right_image_name,
                        WinBoundary& bound) const
{
    const std::string left_img = left_image_path + "/" + left_image_name;
    const std::string right_img = right_image_path + "/" + right_image_name;
    cv::Mat imgL = cv::imread(left_img, cv::IMREAD_GRAYSCALE);
    cv::Mat imgR = cv::imread(right_img, cv::IMREAD_GRAYSCALE);

    imgL = rectifier.rectify(imgL, Rectifier::LEFT);
    imgR = rectifier.rectify(imgR, Rectifier::RIGHT);

    //SGBM
    int mindisparity = 0;
    int ndisparities = 64;
    int SADWindowSize = 5; //blocksize
    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(mindisparity, ndisparities, SADWindowSize);

    int P1 = 8 * imgL.channels() * SADWindowSize * SADWindowSize;
    int P2 = 32 * imgL.channels() * SADWindowSize * SADWindowSize;
    sgbm->setP1(P1);
    sgbm->setP2(P2);
    sgbm->setPreFilterCap(15);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleRange(2);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setDisp12MaxDiff(1);

    cv::Mat disp;
    sgbm->compute(imgL, imgR, disp);        // CV_16S
    disp.convertTo(disp, CV_32F, 1.0 / 16); //除以16得到真实视差值

    // disparity map
    cv::Mat disp8U = cv::Mat(disp.rows, disp.cols, CV_8UC1);
    cv::normalize(disp, disp8U, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::imwrite("disparity.jpg", disp8U);

    // depth_map
    cv::Mat depth = cv::Mat(disp8U.rows, disp8U.cols, CV_32FC1);
    for (int v = 0; v < disp8U.rows; v++)
    {
        for (int u = 0; u < disp8U.cols; u++)
        {
            uchar disp_val = disp8U.ptr<uchar>(v)[u];
            if (disp_val == 0)
                continue;

            float d = fx * baseline / disp_val;
            depth.ptr<float>(v)[u] = d;
        }
    }
    cv::imwrite("depth_before.jpg", depth);
    FillDepthMap32F(depth);
    cv::imwrite("depth_after.jpg", depth);

    // ply_model
    Model model;
    cv::Mat color_map = rectifier.rectify(cv::imread(left_img), Rectifier::LEFT);
    for (int v = 0; v < color_map.rows; v++)
    {
        for (int u = 0; u < color_map.cols; u++)
        {
            double d = depth.ptr<float>(v)[u];
            if (d > 60 || d < 25)
                continue;

            Eigen::Vector3d pos;
            pos.z() = d;
            pos.x() = (u - cx) * pos.z() / fx;
            pos.y() = (v - cy) * pos.z() / fy;
            ModelColor color;
            color.b = color_map.data[v * color_map.step + u * color_map.channels()];
            color.g = color_map.data[v * color_map.step + u * color_map.channels() + 1];
            color.r = color_map.data[v * color_map.step + u * color_map.channels() + 2];
            model.push_back({pos, color});

            bound.wmin(0) = std::min(bound.wmin(0), pos.x());
            bound.wmin(1) = std::min(bound.wmin(1), pos.y());
            bound.wmin(2) = std::min(bound.wmin(2), pos.z());
            bound.wmax(0) = std::max(bound.wmax(0), pos.x());
            bound.wmax(1) = std::max(bound.wmax(1), pos.y());
            bound.wmax(2) = std::max(bound.wmax(2), pos.z());
        }
    }

    return model;
}
