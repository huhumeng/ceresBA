#pragma once

#include <random>
#include <Eigen/Core>

inline double gaussion(double miu, double sigma){ 
    static std::default_random_engine generator;
    static std::normal_distribution<double> distribution(miu, sigma);
    return distribution(generator);
}

inline double uniform(double from, double to){

    static std::default_random_engine generator;
    static std::uniform_real_distribution<double> distribution(from, to);
    return distribution(generator);

}

inline int uniform(int from, int to){
    static std::default_random_engine generator;
    static std::uniform_int_distribution<int> distribution(from, to);
    return distribution(generator);
}

class Camera{
public:
    Camera(double _fx, double _fy, double _cx, double _cy)
    :fx(_fx), fy(_fy), cx(_cx), cy(_cy){}

    inline Eigen::Vector2d project(Eigen::Vector3d& point){
        Eigen::Vector2d p;
        p[0] = fx * point[0] / point[2] + cx;
        p[1] = fy * point[1] / point[2] + cy;
        return p;
    }

private:
    double fx, fy, cx, cy;
};