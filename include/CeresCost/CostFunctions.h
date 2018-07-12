#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <ceres/rotation.h>

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