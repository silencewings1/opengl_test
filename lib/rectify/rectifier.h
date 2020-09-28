#pragma once
#include <opencv2/ml/ml.hpp>
#include <opencv2/opencv.hpp>
#include <string>

class Rectifier
{
public:
    Rectifier(const cv::Size& img_size, const std::string& map_folder)
    {
        assert(img_size.width == 1920 && img_size.height == 1080);
        init_rectify_para(map_folder);
    }

    enum ImgIdx
    {
        LEFT,
        RIGHT,

        INVAILD
    };

    cv::Mat rectify(const cv::Mat& img, const ImgIdx& id = INVAILD) const
    {
        cv::Mat res;
        switch (id)
        {
        case ImgIdx::LEFT:
            cv::remap(img, res, left_map1, left_map2, cv::INTER_LINEAR);
            break;
        case ImgIdx::RIGHT:
            cv::remap(img, res, right_map1, right_map2, cv::INTER_LINEAR);
            break;
        default:
            return img;
        }

        return res;
    }

private:
    void init_rectify_para(const std::string& map_folder)
    {
        cv::Ptr<cv::ml::TrainData> train_data;

        train_data = cv::ml::TrainData::loadFromCSV(map_folder + "/xmap1.csv", 0);
        left_map1 = train_data->getTrainSamples();
        train_data = cv::ml::TrainData::loadFromCSV(map_folder + "/ymap1.csv", 0);
        left_map2 = train_data->getTrainSamples();
        train_data = cv::ml::TrainData::loadFromCSV(map_folder + "/xmap2.csv", 0);
        right_map1 = train_data->getTrainSamples();
        train_data = cv::ml::TrainData::loadFromCSV(map_folder + "/ymap2.csv", 0);
        right_map2 = train_data->getTrainSamples();
    }

private:
    cv::Mat left_map1;
    cv::Mat left_map2;
    cv::Mat right_map1;
    cv::Mat right_map2;
};