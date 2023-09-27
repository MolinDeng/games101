#include "Triangle.hpp"
#include "rasterizer.hpp"
#include <eigen3/Eigen/Eigen>
#include <iostream>
#include <opencv2/opencv.hpp>

#define W 1000
#define H 1000

constexpr double MY_PI = 3.1415926;

Eigen::Matrix4f get_rotation(Vector3f axis, float angle) {
    Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
    Eigen::Matrix3f NNT = axis * axis.transpose();
    Eigen::Matrix3f A_star;
    A_star << 0, -axis[2], axis[1],
        axis[2], 0, -axis[0],
        -axis[1], axis[0], 0;
    Eigen::Matrix3f R = cos(angle / 180 * MY_PI) * I + (1 - cos(angle / 180 * MY_PI)) * NNT + sin(angle / 180 * MY_PI) * A_star;

    Eigen::Matrix4f rotate = Eigen::Matrix4f::Identity();
    rotate.block(0, 0, 3, 3) = R;
    return rotate;
}

Eigen::Matrix4f get_view_matrix(Eigen::Vector3f eye_pos) {
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();

    Eigen::Matrix4f translate;
    translate << 1, 0, 0, -eye_pos[0], 0, 1, 0, -eye_pos[1], 0, 0, 1,
        -eye_pos[2], 0, 0, 0, 1;

    view = translate * view;

    return view;
}

Eigen::Matrix4f get_model_matrix(float rotation_angle) {
    Eigen::Matrix4f model = Eigen::Matrix4f::Identity();

    // Eigen::Matrix4f rotate;
    // rotate << cos(rotation_angle / 180 * MY_PI), -sin(rotation_angle / 180 * MY_PI), 0, 0,
    //     sin(rotation_angle / 180 * MY_PI), cos(rotation_angle / 180 * MY_PI), 0, 0,
    //     0, 0, 1, 0,
    //     0, 0, 0, 1;
    model = get_rotation(Eigen::Vector3f(0, 1, 0), rotation_angle) * model;

    return model;
}

Eigen::Matrix4f get_projection_matrix(float eye_fov, float aspect_ratio,
                                      float zNear, float zFar) {
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();

    float l, r, t, b, n, f;
    n = -zNear;
    f = -zFar;
    t = tan(eye_fov / 2 / 180 * MY_PI) * abs(n);
    b = -t;
    r = t * aspect_ratio;
    l = -r;

    Eigen::Matrix4f persp2ortho, ortho_scale, orthor_trans;
    persp2ortho << n, 0, 0, 0,
        0, n, 0, 0,
        0, 0, n + f, -n * f,
        0, 0, 1, 0;
    ortho_scale << 2 / (r - l), 0, 0, 0,
        0, 2 / (t - b), 0, 0,
        0, 0, 2 / (n - f), 0,
        0, 0, 0, 1;
    orthor_trans << 1, 0, 0, -(l + r) / 2,
        0, 1, 0, -(t + b) / 2,
        0, 0, 1, -(n + f) / 2,
        0, 0, 0, 1;
    projection = ortho_scale * orthor_trans * persp2ortho * projection;

    return projection;
}

int main(int argc, const char **argv) {
    float angle = 0;
    bool command_line = false;
    std::string filename = "output.png";

    if (argc >= 3) {
        command_line = true;
        angle = std::stof(argv[2]); // -r by default
        if (argc == 4) {
            filename = std::string(argv[3]);
        } else
            return 0;
    }

    rst::rasterizer r(W, H);

    Eigen::Vector3f eye_pos = {0, 0, 5};

    std::vector<Eigen::Vector3f> pos{{2, 0, -1}, {0, 2, -1}, {-2, 0, -1}};

    std::vector<Eigen::Vector3i> ind{{0, 1, 2}};

    auto pos_id = r.load_positions(pos);
    auto ind_id = r.load_indices(ind);

    int key = 0;
    int frame_count = 0;

    if (command_line) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);
        cv::Mat image(W, H, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);

        cv::imwrite(filename, image);

        return 0;
    }

    while (key != 27) {
        r.clear(rst::Buffers::Color | rst::Buffers::Depth);

        r.set_model(get_model_matrix(angle));
        r.set_view(get_view_matrix(eye_pos));
        r.set_projection(get_projection_matrix(45, 1, 0.1, 50));

        r.draw(pos_id, ind_id, rst::Primitive::Triangle);

        cv::Mat image(W, H, CV_32FC3, r.frame_buffer().data());
        image.convertTo(image, CV_8UC3, 1.0f);
        cv::imshow("image", image);
        key = cv::waitKey(10);

        std::cout << "frame count: " << frame_count++ << '\n';

        if (key == 'a') {
            angle += 10;
        } else if (key == 'd') {
            angle -= 10;
        }
    }

    return 0;
}
