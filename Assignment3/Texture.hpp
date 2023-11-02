//
// Created by LEI XU on 4/27/19.
//

#ifndef RASTERIZER_TEXTURE_H
#define RASTERIZER_TEXTURE_H
#include "global.hpp"
#include <eigen3/Eigen/Eigen>
#include <opencv2/opencv.hpp>
class Texture {
private:
    cv::Mat image_data;

public:
    Texture(const std::string &name) {
        image_data = cv::imread(name);
        cv::cvtColor(image_data, image_data, cv::COLOR_RGB2BGR);
        width = image_data.cols;
        height = image_data.rows;
    }

    int width, height;

    Eigen::Vector3f getColor(float u, float v) {
        return getColorBilinear(u, v);
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto color = image_data.at<cv::Vec3b>(v_img, u_img);
        return Eigen::Vector3f(color[0], color[1], color[2]);
    }

    Eigen::Vector3f getColorBilinear(float u, float v) {
        auto u_img = u * width;
        auto v_img = (1 - v) * height;
        auto u_left = (int)u_img;
        auto u_right = std::min(u_left + 1, width);
        auto v_top = (int)v_img;
        auto v_bottom = std::min(v_top + 1, height);
        auto u_ratio = u_img - u_left;
        auto v_ratio = v_img - v_bottom;
        auto color_top_left = image_data.at<cv::Vec3b>(v_top, u_left);
        auto color_top_right = image_data.at<cv::Vec3b>(v_top, u_right);
        auto color_bottom_left = image_data.at<cv::Vec3b>(v_bottom, u_left);
        auto color_bottom_right = image_data.at<cv::Vec3b>(v_bottom, u_right);

        auto color_top = color_top_left + (color_top_right - color_top_left) * u_ratio;
        auto color_bottom = color_bottom_left + (color_bottom_right - color_bottom_left) * u_ratio;
        auto color = color_bottom + (color_top - color_bottom) * v_ratio;

        return Eigen::Vector3f(color[0], color[1], color[2]);
    }
};
#endif // RASTERIZER_TEXTURE_H
