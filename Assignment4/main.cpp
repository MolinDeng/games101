#include <chrono>
#include <iostream>
#include <opencv2/opencv.hpp>

std::vector<cv::Point2f> control_points;

void mouse_handler(int event, int x, int y, int flags, void *userdata) {
    if (event == cv::EVENT_LBUTTONDOWN && control_points.size() < 4) {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", "
                  << y << ")" << '\n';
        control_points.emplace_back(x, y);
    }
}

void naive_bezier(const std::vector<cv::Point2f> &points, cv::Mat &window) {
    auto &p_0 = points[0];
    auto &p_1 = points[1];
    auto &p_2 = points[2];
    auto &p_3 = points[3];

    for (double t = 0.0; t <= 1.0; t += 0.001) {
        auto point = std::pow(1 - t, 3) * p_0 + 3 * t * std::pow(1 - t, 2) * p_1 +
                     3 * std::pow(t, 2) * (1 - t) * p_2 + std::pow(t, 3) * p_3;

        window.at<cv::Vec3b>(point.y, point.x)[2] = 255;
    }
}

cv::Point2f recursive_bezier(std::vector<cv::Point2f> &control_points, int n, float t) {
    // Implement de Casteljau's algorithm
    if (n == 1)
        return control_points[0];

    for (int i = 0; i < n - 1; ++i) {
        auto point = (1 - t) * control_points[i] + t * control_points[i + 1];
        control_points[i].x = point.x;
        control_points[i].y = point.y;
    }

    return recursive_bezier(control_points, n - 1, t);
}

void bezier(const std::vector<cv::Point2f> &control_points, cv::Mat &window) {
    // make a copy of control points
    std::vector<cv::Point2f> points = control_points;

    for (double t = 0.0; t <= 1.0; t += 0.001) {
        for (int i = 0; i < points.size(); i++) {
            points[i].x = control_points[i].x;
            points[i].y = control_points[i].y;
        }

        auto point = recursive_bezier(points, points.size(), t);

        window.at<cv::Vec3b>(point.y, point.x)[1] = 255;
    }
}

int main() {
    cv::Mat window = cv::Mat(700, 700, CV_8UC3, cv::Scalar(0));
    cv::cvtColor(window, window, cv::COLOR_BGR2RGB);
    cv::namedWindow("Bezier Curve", cv::WINDOW_AUTOSIZE);

    cv::setMouseCallback("Bezier Curve", mouse_handler, nullptr);

    int key = -1;
    while (key != 27) {
        for (auto &point : control_points) {
            cv::circle(window, point, 3, {255, 255, 255}, 3);
        }

        if (control_points.size() == 4) {
            naive_bezier(control_points, window);
            bezier(control_points, window);

            cv::imshow("Bezier Curve", window);
            cv::imwrite("my_bezier_curve.png", window);
            key = cv::waitKey(0);

            return 0;
        }

        cv::imshow("Bezier Curve", window);
        key = cv::waitKey(20);
    }

    return 0;
}
