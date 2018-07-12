#pragma once

#include <random>
#include <Eigen/Core>
#include <ceres/rotation.h>

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

class generatePointAndPose{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<Eigen::Vector3d> p3ds, pc3ds;
    std::vector<Eigen::Vector3d> p2ds;
    std::vector<unsigned char> status;
    
    Eigen::Vector3d r, t;
    
    // --------------------------------------------------
    void setPose(double* rotation, double* translation){
        r = Eigen::Vector3d(rotation);
        t = Eigen::Vector3d(translation);
    }

    void genWorldPoint(){
        for(size_t i=0; i<point_num; ++i){
            Eigen::Vector3d p(
                uniform(0.0, 20.0), uniform(0.0, 20.0), uniform(0.0, 20.0)
            );
            p3ds.push_back(p);
        }
    }

    void genCameraPoint(){
        
        for(size_t i=0; i<point_num; ++i){
            double pcamera[3];
            ceres::AngleAxisRotatePoint(r.data(), p3ds[i].data(), pcamera);
            Eigen::Vector3d pc(pcamera);
            pc += t;
            pc3ds.push_back(pc);
        }
        
    }

    void genAddPoseNoise(){
        
    }

private:
    const size_t point_num = 30;
};