#pragma once

#include "Random.h"
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <ceres/rotation.h>

class CameraModel{
public:

    CameraModel(double _fx, double _fy, double _cx, double _cy, double _height, double _width)
    :fx(_fx), fy(_fy), cx(_cx), cy(_cy), height(_height), width(_width){}

    ~CameraModel(){}

    Eigen::Vector2d project(Eigen::Vector3d& point){
        Eigen::Vector2d p;
        p[0] = fx * point[0] / point[2] + cx;
        p[1] = fy * point[1] / point[2] + cy;
        return p;
    }

    bool isInCamera(const Eigen::Vector2d& uv) const {
        if(uv[0] < 0 || uv[1] < 0)
            return false;
        if(uv[0] > width || uv[1] > height)
            return false;
        
        return true;
    }

     bool isInCamera(const double& x, const double& y) const {
        if(x < 0 || y < 0)
            return false;
        if(x > width || y > height)
            return false;
        
        return true;
    }

private:
    double fx, fy, cx, cy;
    double height, width;
};

class GenPointWithPoseKnown{

public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    std::vector<Eigen::Vector3d> p3ds, pc3ds, p3dnoise;
    std::vector<Eigen::Vector2d> p2ds;
    std::vector<unsigned char> status;
        
    // --------------------------------------------------
    GenPointWithPoseKnown(double* rotation, double* translation, int num_point)
    :r(Eigen::Vector3d(rotation)), 
     t(Eigen::Vector3d(translation)), 
     point_num(num_point){
        // p3ds.reserve(num_point);
        // pc3ds.reserve(num_point);
        // p2ds.reserve(num_point);
        // status = std::vector<unsigned char>(0, num_point);
        camera = new CameraModel(520.9, 521.0, 325.1, 249.7, 640.0, 480.0);
        genPoint();
    }

    ~GenPointWithPoseKnown(){
        delete camera;
    }

    inline const size_t& N()const{return point_num;}
    
    

    // void genAddPoseNoise(){
    //     r += Eigen::Vector3d(gaussion(0, 1), gaussion(0, 1), gaussion(0, 1));
    //     t += Eigen::Vector3d(gaussion(0, 1), gaussion(0, 1), gaussion(0, 1));
    // }


private:

    Eigen::Vector3d r, t;
    const size_t point_num;
    //
    // +325.1;
    // const T v=y*521.0+249.7;
    CameraModel* camera;
    // --------------------------------------------------------------
    void genPoint(){
    
        for(size_t i=0; i < point_num; ++i){
            
            // 生成一个P_world
            Eigen::Vector3d po(
                uniform(0.0, 20.0), uniform(0.0, 20.0), uniform(0.0, 20.0)
            );

            // 噪声
            Eigen::Vector3d noise(
                gaussion(0.0, 0.1), gaussion(0.0, 0.1), gaussion(0.0, 0.1)
            );

            // 读取的3D点的值
            Eigen::Vector3d p = po + noise;

            double pcamera[3];
            ceres::AngleAxisRotatePoint(r.data(), po.data(), pcamera);
            Eigen::Vector3d pc(pcamera);
            // 理论上的camera的3D位置
            pc += t;
            
            if(pc[2] <= 0)
                continue;
            Eigen::Vector2d uv = camera->project(pc);
            // uv含有噪声
            Eigen::Vector2d noise2(
                gaussion(0.0, 0.5), gaussion(0.0, 0.5)
            );
            uv += noise2;
            if(!camera->isInCamera(uv))
                continue;
            p2ds.push_back(uv);
            p3ds.push_back(po);    // 原始的真实3D点
            p3dnoise.push_back(p); // 读取的3D点
            pc3ds.push_back(pc);   // 真实的Pc
        }
    }
};