# Games 101

Personal study repo for UCSB Games 101

## Assignment 1

Implement MVP transform within a simplified Rasterization pipeline

```C++
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

    Eigen::Matrix4f rotate;
    rotate << cos(rotation_angle / 180 * MY_PI), -sin(rotation_angle / 180 * MY_PI), 0, 0,
        sin(rotation_angle / 180 * MY_PI), cos(rotation_angle / 180 * MY_PI), 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    model = rotate * model;

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
```

## Bonus

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
