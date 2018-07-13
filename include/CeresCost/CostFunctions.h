#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <ceres/rotation.h>

#include <glog/logging.h>

// ICP
struct Project3DTo3DPoseOnlyCost{

    Project3DTo3DPoseOnlyCost(Eigen::Vector3d& _p1, Eigen::Vector3d& _p2)
    :p1(_p1), p2(_p2){}

    ~Project3DTo3DPoseOnlyCost(){}

    template<typename T>
    bool operator()(const T* const rotation, const T* const translat, T* residual) const
    {
        T p_r[3];
        T p_c[3];

        p_r[0] = T(p1[0]);
        p_r[1] = T(p1[1]);
        p_r[2] = T(p1[2]);

        ceres::AngleAxisRotatePoint(rotation, p_r, p_c);

        p_c[0] = p_c[0] + translat[0];
        p_c[1] = p_c[1] + translat[1];
        p_c[2] = p_c[2] + translat[2];

        residual[0] = T(p2[0]) - p_c[0];
        residual[1] = T(p2[1]) - p_c[1];
        residual[2] = T(p2[2]) - p_c[2];

        return true;
    }
private:
    Eigen::Vector3d p1, p2;

};

// PnP
struct Project3DTo2DPoseOnlyCost{

    Project3DTo2DPoseOnlyCost(Eigen::Vector3d& _p1, Eigen::Vector2d& _p2, 
                              double _fx, double _fy, double _cx, double _cy)
    :p1(_p1), p2(_p2), fx(_fx), fy(_fy), cx(_cx), cy(_cy){}

    ~Project3DTo2DPoseOnlyCost(){}

    template<typename T>
    bool operator()(const T* const rotation, const T* const translat, T* residual) const
    {
        T p_r[3];
        T p_c[3];

        p_r[0] = T(p1[0]);
        p_r[1] = T(p1[1]);
        p_r[2] = T(p1[2]);

        ceres::AngleAxisRotatePoint(rotation, p_r, p_c);

        p_c[0] = p_c[0] + translat[0];
        p_c[1] = p_c[1] + translat[1];
        p_c[2] = p_c[2] + translat[2];

        T uv[2];
        uv[0] = fx / p_c[2] * p_c[0] + cx;
        uv[1] = fy / p_c[2] * p_c[1] + cy;

        residual[0] = T(p2[0]) - uv[0];
        residual[1] = T(p2[1]) - uv[1];

        return true;
    }
private:
    Eigen::Vector3d p1;
    Eigen::Vector2d p2;

    const double fx, fy, cx, cy;

};

// PnP with 3D point
struct Project3DTo2DCost{

    Project3DTo2DCost(Eigen::Vector2d& _measure, 
                      double _fx, double _fy, double _cx, double _cy)
    :measure(_measure), fx(_fx), fy(_fy), cx(_cx), cy(_cy){}

    ~Project3DTo2DCost(){}

    template<typename T>
    bool operator()(const T* const rotation, const T* const translat, 
                    const T* const p_world, T* residual) const
    {

        T p_camera[3];

        ceres::AngleAxisRotatePoint(rotation, p_world, p_camera);

        p_camera[0] = p_camera[0] + translat[0];
        p_camera[1] = p_camera[1] + translat[1];
        p_camera[2] = p_camera[2] + translat[2];
        
        T uvuv[2];
        uvuv[0] = fx / p_camera[2] * p_camera[0] + cx;
        uvuv[1] = fy / p_camera[2] * p_camera[1] + cy;

        // T uv[2];
        // uv[0] = T(measure[0]);
        // uv[1] = T(measure[1]);

        residual[0] = uvuv[0] - T(measure[0]);
        residual[1] = uvuv[1] - T(measure[1]);
        // LOG(INFO) << residual[0] << " " << residual[1];
        return true;
    }
private:
    // Eigen::Vector3d p1;
    Eigen::Vector2d measure;

    const double fx, fy, cx, cy;

};
