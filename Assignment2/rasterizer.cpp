// clang-format off
//
// Created by goksu on 4/6/19.
//

#include <algorithm>
#include <vector>
#include "rasterizer.hpp"
#include <opencv2/opencv.hpp>
#include <math.h>


rst::pos_buf_id rst::rasterizer::load_positions(const std::vector<Eigen::Vector3f> &positions)
{
    auto id = get_next_id();
    pos_buf.emplace(id, positions);

    return {id};
}

rst::ind_buf_id rst::rasterizer::load_indices(const std::vector<Eigen::Vector3i> &indices)
{
    auto id = get_next_id();
    ind_buf.emplace(id, indices);

    return {id};
}

rst::col_buf_id rst::rasterizer::load_colors(const std::vector<Eigen::Vector3f> &cols)
{
    auto id = get_next_id();
    col_buf.emplace(id, cols);

    return {id};
}

auto to_vec4(const Eigen::Vector3f& v3, float w = 1.0f)
{
    return Vector4f(v3.x(), v3.y(), v3.z(), w);
}

static float cross2D(const Vector2f& v1, const Vector2f& v2)
{
    return v1.x()*v2.y() - v1.y()*v2.x();
}

static bool insideTriangle(float x, float y, const Vector3f* _v)
{   
    Vector2f p(x,y);
    bool ccw1 = cross2D(_v[1].head<2>() - _v[0].head<2>(), p - _v[0].head<2>()) >= 0;
    bool ccw2 = cross2D(_v[2].head<2>() - _v[1].head<2>(), p - _v[1].head<2>()) >= 0;
    bool ccw3 = cross2D(_v[0].head<2>() - _v[2].head<2>(), p - _v[2].head<2>()) >= 0;
    return ccw1 == ccw2 && ccw2 == ccw3;
}

static std::tuple<float, float, float> computeBarycentric2D(float x, float y, const Vector3f* v)
{
    float c1 = (x*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*y + v[1].x()*v[2].y() - v[2].x()*v[1].y()) / (v[0].x()*(v[1].y() - v[2].y()) + (v[2].x() - v[1].x())*v[0].y() + v[1].x()*v[2].y() - v[2].x()*v[1].y());
    float c2 = (x*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*y + v[2].x()*v[0].y() - v[0].x()*v[2].y()) / (v[1].x()*(v[2].y() - v[0].y()) + (v[0].x() - v[2].x())*v[1].y() + v[2].x()*v[0].y() - v[0].x()*v[2].y());
    float c3 = (x*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*y + v[0].x()*v[1].y() - v[1].x()*v[0].y()) / (v[2].x()*(v[0].y() - v[1].y()) + (v[1].x() - v[0].x())*v[2].y() + v[0].x()*v[1].y() - v[1].x()*v[0].y());
    return {c1,c2,c3};
}

void rst::rasterizer::draw(pos_buf_id pos_buffer, ind_buf_id ind_buffer, col_buf_id col_buffer, Primitive type)
{
    auto& buf = pos_buf[pos_buffer.pos_id];
    auto& ind = ind_buf[ind_buffer.ind_id];
    auto& col = col_buf[col_buffer.col_id];

    float f1 = (50 - 0.1) / 2.0;
    float f2 = (50 + 0.1) / 2.0;

    Eigen::Matrix4f mvp = projection * view * model;
    for (auto& i : ind)
    {
        Triangle t;
        Eigen::Vector4f v[] = {
                mvp * to_vec4(buf[i[0]], 1.0f),
                mvp * to_vec4(buf[i[1]], 1.0f),
                mvp * to_vec4(buf[i[2]], 1.0f)
        };
        //Homogeneous division
        for (auto& vec : v) {
            vec /= vec.w();
        }
        //Viewport transformation
        for (auto & vert : v)
        {
            vert.x() = 0.5*width*(vert.x()+1.0);
            vert.y() = 0.5*height*(vert.y()+1.0);
            vert.z() = vert.z() * f1 + f2;
        }

        for (int i = 0; i < 3; ++i)
        {
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
            t.setVertex(i, v[i].head<3>());
        }

        auto col_x = col[i[0]];
        auto col_y = col[i[1]];
        auto col_z = col[i[2]];

        t.setColor(0, col_x[0], col_x[1], col_x[2]);
        t.setColor(1, col_y[0], col_y[1], col_y[2]);
        t.setColor(2, col_z[0], col_z[1], col_z[2]);

        // rasterize_triangle_ssaa(t);
        rasterize_triangle_ssaa2(t);
        // rasterize_triangle(t);
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle(const Triangle& t) {
    auto v = t.toVector4();
    // Find out the bounding box of current triangle.
    int xmin = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    xmin = std::max(0, xmin);
    int xmax = std::max(v[0].x(), std::max(v[1].x(), v[2].x()))+1;
    xmax = std::min(width-1, xmax);
    int ymin = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    ymin = std::max(0, ymin);
    int ymax = std::max(v[0].y(), std::max(v[1].y(), v[2].y()))+1;
    ymax = std::min(height-1, ymax);
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int x = xmin; x <= xmax; ++x) {
        for (int y = ymin; y <= ymax; ++y) {
            if (insideTriangle(x, y, t.v)) {
                // If so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(x, y, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;
                if (z_interpolated > depth_buf[get_index(x, y)]) {
                    depth_buf[get_index(x, y)] = z_interpolated;
                    set_pixel(Eigen::Vector3f(x, y, z_interpolated), t.getColor());
                }
            }
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle_ssaa(const Triangle& t) {
    auto v = t.toVector4();
    // Find out the bounding box of current triangle.
    int xmin = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    xmin = std::max(0, xmin);
    int xmax = std::max(v[0].x(), std::max(v[1].x(), v[2].x()))+1;
    xmax = std::min(width-1, xmax);
    int ymin = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    ymin = std::max(0, ymin);
    int ymax = std::max(v[0].y(), std::max(v[1].y(), v[2].y()))+1;
    ymax = std::min(height-1, ymax);

    std::vector<float> steps{0.75, 0.25, 0.25, 0.75, 0.75};
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int x = xmin; x <= xmax; ++x) {
        for (int y = ymin; y <= ymax; ++y) {
            for (int i = 0; i < 4; ++i) {
                float xf = x + steps[i];
                float yf = y + steps[i+1];
                // reset frame buffer
                if (insideTriangle(xf, yf, t.v)) {
                    // If so, use the following code to get the interpolated z value.
                    auto[alpha, beta, gamma] = computeBarycentric2D(xf, yf, t.v);
                    float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                    float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                    z_interpolated *= w_reciprocal;
                    if (z_interpolated > depth_buf_ssaa[get_index(x, y) * 4 + i]) {
                        depth_buf_ssaa[get_index(x, y) * 4 + i] = z_interpolated;
                        frame_buf_ssaa[get_index(x, y) * 4 + i] = t.getColor();
                    }
                }
            }
            Eigen::Vector3f color = Eigen::Vector3f(0,0,0);
            for (int i = 0; i < 4; ++i) 
                color += frame_buf_ssaa[get_index(x, y) * 4 + i] * 0.25;
            frame_buf[get_index(x, y)] = color;
        }
    }
}

//Screen space rasterization
void rst::rasterizer::rasterize_triangle_ssaa2(const Triangle& t) {
    auto v = t.toVector4();
    // Find out the bounding box of current triangle.
    int xmin = std::min(v[0].x(), std::min(v[1].x(), v[2].x()));
    xmin = std::max(0, xmin);
    int xmax = std::max(v[0].x(), std::max(v[1].x(), v[2].x()))+1;
    xmax = std::min(width-1, xmax);
    int ymin = std::min(v[0].y(), std::min(v[1].y(), v[2].y()));
    ymin = std::max(0, ymin);
    int ymax = std::max(v[0].y(), std::max(v[1].y(), v[2].y()))+1;
    ymax = std::min(height-1, ymax);

    std::vector<float> steps{0.75, 0.25, 0.25, 0.75, 0.75};
    // iterate through the pixel and find if the current pixel is inside the triangle
    for (int x = xmin; x <= xmax; ++x) {
        for (int y = ymin; y <= ymax; ++y) {
            int cnt = 0, index = get_index(x, y);;
            for (int i = 0; i < 4; ++i) {
                float xf = x + steps[i];
                float yf = y + steps[i+1];
                // reset frame buffer
                cnt += insideTriangle(xf, yf, t.v);
            }

            // find if the current pixel is inside the triangle
            if(insideTriangle(x+0.5, y+0.5, t.v)) {
                // if so, use the following code to get the interpolated z value.
                auto[alpha, beta, gamma] = computeBarycentric2D(x+0.5, y+0.5, t.v);
                float w_reciprocal = 1.0/(alpha / v[0].w() + beta / v[1].w() + gamma / v[2].w());
                float z_interpolated = alpha * v[0].z() / v[0].w() + beta * v[1].z() / v[1].w() + gamma * v[2].z() / v[2].w();
                z_interpolated *= w_reciprocal;

                // set the current pixel (use the set_pixel function) 
                // to the color of the triangle (use getColor function)
                // if it should be painted.
                if(z_interpolated > depth_buf[index]) {
                    Eigen::Vector3f p(x, y, z_interpolated);
                    Eigen::Vector3f color = t.getColor()*(cnt/4.0)+((4-cnt)/4.0)*frame_buf[index];
                    set_pixel(p, color);
                    depth_buf[index] = z_interpolated;
                }   
            }
            if(cnt < 4) {
                depth_buf[index] = -std::numeric_limits<float>::infinity();
            }
        }
    }
}

void rst::rasterizer::set_model(const Eigen::Matrix4f& m) 
{
    model = m;
}

void rst::rasterizer::set_view(const Eigen::Matrix4f& v)
{
    view = v;
}

void rst::rasterizer::set_projection(const Eigen::Matrix4f& p)
{
    projection = p;
}

void rst::rasterizer::clear(rst::Buffers buff)
{
    if ((buff & rst::Buffers::Color) == rst::Buffers::Color)
    {
        std::fill(frame_buf.begin(), frame_buf.end(), Eigen::Vector3f{0, 0, 0});
        std::fill(frame_buf_ssaa.begin(), frame_buf_ssaa.end(), Eigen::Vector3f{0, 0, 0});
    }
    if ((buff & rst::Buffers::Depth) == rst::Buffers::Depth)
    {
        std::fill(depth_buf.begin(), depth_buf.end(), -std::numeric_limits<float>::infinity());
        std::fill(depth_buf_ssaa.begin(), depth_buf_ssaa.end(), -std::numeric_limits<float>::infinity());
    }
}

rst::rasterizer::rasterizer(int w, int h) : width(w), height(h)
{
    frame_buf.resize(w * h);
    depth_buf.resize(w * h);
    frame_buf_ssaa.resize(w * h * 4);
    depth_buf_ssaa.resize(w * h * 4);
}

int rst::rasterizer::get_index(int x, int y)
{
    return (height-1-y)*width + x;
}

void rst::rasterizer::set_pixel(const Eigen::Vector3f& point, const Eigen::Vector3f& color)
{
    //old index: auto ind = point.y() + point.x() * width;
    auto ind = (height-1-point.y())*width + point.x();
    frame_buf[ind] = color;

}

// clang-format on