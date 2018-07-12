#pragma once

#include "Random.h"
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <ceres/rotation.h>

class CameraModel{
public:

    CameraModel(double _fx, double _fy, double _cx, double _cy)
    :fx(_fx), fy(_fy), cx(_cx), cy(_cy){}

    ~CameraModel(){}

    Eigen::Vector2d project(Eigen::Vector3d& point){
        Eigen::Vector2d p;
        p[0] = fx * point[0] / point[2] + cx;
        p[1] = fy * point[1] / point[2] + cy;
        return p;
    }

private:
    double fx, fy, cx, cy;
};

class GenPointWithPoseKnown{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<Eigen::Vector3d> p3ds, pc3ds;
    std::vector<Eigen::Vector3d> p2ds;
    std::vector<unsigned char> status;
        
    // --------------------------------------------------
    GenPointWithPoseKnown(double* rotation, double* translation, int num_point)
    :r(Eigen::Vector3d(rotation)), 
     t(Eigen::Vector3d(translation)), 
     point_num(num_point){
        genPoint();
    }

    ~GenPointWithPoseKnown(){}

    inline const size_t& N()const{return point_num;}
    
    

    // void genAddPoseNoise(){
    //     r += Eigen::Vector3d(gaussion(0, 1), gaussion(0, 1), gaussion(0, 1));
    //     t += Eigen::Vector3d(gaussion(0, 1), gaussion(0, 1), gaussion(0, 1));
    // }


private:

    Eigen::Vector3d r, t;
    const size_t point_num;
    // --------------------------------------------------------------
    void genPoint(){
    
        for(size_t i=0; i < point_num; ++i){
            
            // 生成一个P_world
            Eigen::Vector3d p(
                uniform(0.0, 20.0), uniform(0.0, 20.0), uniform(0.0, 20.0)
            );

            double pcamera[3];
            ceres::AngleAxisRotatePoint(r.data(), p.data(), pcamera);
            Eigen::Vector3d pc(pcamera);
            pc += t;

            p3ds.push_back(p);
            pc3ds.push_back(pc);
        }
    }
};