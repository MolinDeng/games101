# Games 101

Personal study repo for UCSB Games 101

## Assignment 1

<p align="center">
    <img src="misc/1.jpg" style="height: 500px; width:500px;"/>
</p>

### Bonus

Rotation by angle $\alpha$ around axis $\vec{\text{n}} = (n_x, n_y, n_z)$

* By default, any $\text{n}$ will cross (0, 0, 0)

```C++
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
```

## Assignment 2

* We need to change the initialization of depth buffer in `clear` function

```C++
std::fill(depth_buf.begin(), depth_buf.end(), -std::numeric_limits<float>::infinity());
```

<p align="center">
    <img src="misc/2.jpg" style="height: 500px; width:500px;"/>
</p>

### Bonus

```C++
void rasterize_triangle_ssaa(const Triangle &t);
void rasterize_triangle_ssaa2(const Triangle &t);
```

* Method 2: for a pixel on edges (cnt < 4), each pass, we clear its depth buffer to ensure both colors will be painted
* Reference: <https://zhuanlan.zhihu.com/p/454001952>

<p align="center">
    <img src="misc/2-bonus.jpg" style="height: 500px; width:500px;"/> <img src="misc/2-compare.jpg" style="height: 300px; width:600px;"/>
</p>

## Assignment 3

* Change the initialization of depth buffer in `clear` function

```C++
std::fill(depth_buf.begin(), depth_buf.end(), -std::numeric_limits<float>::infinity());
```

* `getColorBilinear`

```C++
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
```

<p align="center">
    <img src="misc/3/output1.png" style="height: 150px; width:150px;"/> <img src="misc/3/output2.png" style="height: 150px; width:150px;"/> <img src="misc/3/output3.png" style="height: 150px; width:150px;"/> <img src="misc/3/output4.png" style="height: 150px; width:150px;"/> <img src="misc/3/output5.png" style="height: 150px; width:150px;"/>
</p>
<p align="center">
<img src="misc/3/bilinear.png" style="height: auto; width:auto;"/>
</p>
