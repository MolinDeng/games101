cmake_minimum_required(VERSION 3.10)
project(RayTracing)

set(CMAKE_CXX_STANDARD 17)

# Set the compiler to clang++ from LLVM
set(CMAKE_CXX_COMPILER "/usr/local/opt/llvm/bin/clang++")

# Add the compile flag -fopenmp
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -O3")

add_executable(RayTracing main.cpp Object.hpp Vector.cpp Vector.hpp Sphere.hpp global.hpp Triangle.hpp Scene.cpp
        Scene.hpp Light.hpp AreaLight.hpp BVH.cpp BVH.hpp Bounds3.hpp Ray.hpp Material.hpp Intersection.hpp
        Renderer.cpp Renderer.hpp)
        
