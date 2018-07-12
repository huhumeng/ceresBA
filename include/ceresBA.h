#pragma once

#include <ceres/ceres.h>
#include <Eigen/Core>
#include <ceres/rotation.h>

// struct Project3DTo2DPoseOnlyCost
// {
//     Project3DTo2DPoseOnlyCost(Eigen::Vector3d& p1, Eigen::Vector2d& p2) : _p1(p1),_p2(p2){}
//     template<typename T>
//     bool operator()(const T* const r, const T* const t, T* residual) const
//     {
//         T p_1[3];
//         T p_2[3];

//         p_1[0] = T(_p1.x);
//         p_1[1] = T(_p1.y);
//         p_1[2] = T(_p1.z);
        
//         ceres::AngleAxisRotatePoint(r, p_1, p_2);
        
//         p_2[0] = p_2[0] + t[0];
//         p_2[1] = p_2[1] + t[1];
//         p_2[2] = p_2[2] + t[2];

//         const T x = p_2[0]/p_2[2];
//         const T y = p_2[1]/p_2[2];
//         const T u = x * 520.9 + 325.1;
//         const T v = y * 521.0 + 249.7;
        
//         T p_3[2];
//         p_3[0] = T(_p2.x);
//         p_3[1] = T(_p2.y);
        
//         // const T x1=p_3[0]/p_3[2];
//         // const T y1=p_3[1]/p_3[2];
//         // const T u1=x1*520.9+325.1;
//         // const T v1=y1*521.0+249.7;
//         residual[0] = u - p_3[0];
//         residual[1] = v - p_3[1];
//         return true;
//     }

//     // ceres::CostFunction* Create(Eigen::Vector3d& p1, Eigen::Vector2d& p2){
//     //     return (
//     //         new ceres::AutoDiffCostFunction<Project3DTo2DPoseOnlyCost, 2, 3, 3>
//     //         (
//     //             new Project3DTo2DPoseOnlyCost(p1, p2)
//     //         )
//     //     );
//     // }

// private:
//     Eigen::Vector3d _p1;
//     Eigen::Vector2d _p2;
// };

struct Project3DTo3DPoseOnlyCost{

    Project3DTo3DPoseOnlyCost(Eigen::Vector3d& p1, Eigen::Vector3d& p2):_p1(p1),_p2(p2){}
    template<typename T>
    bool operator()(const T* const cere_r,const T* const cere_t,T* residual)const
    {
        T p_1[3];
        T p_2[3];
        p_1[0]=T(_p1[0]);
        p_1[1]=T(_p1[1]);
        p_1[2]=T(_p1[2]);
        ceres::AngleAxisRotatePoint(cere_r,p_1,p_2);
        p_2[0]=p_2[0]+cere_t[0];
        p_2[1]=p_2[1]+cere_t[1];
        p_2[2]=p_2[2]+cere_t[2];
        const T x=p_2[0]/p_2[2];
        const T y=p_2[1]/p_2[2];
        const T u=x*520.9+325.1;
        const T v=y*521.0+249.7;
        T p_3[3];
        p_3[0]=T(_p2[0]);
        p_3[1]=T(_p2[1]);
        p_3[2]=T(_p2[2]);
        const T x1=p_3[0]/p_3[2];
        const T y1=p_3[1]/p_3[2];
        const T u1=x1*520.9+325.1;
        const T v1=y1*521.0+249.7;
        residual[0]=u-u1;
        residual[1]=v-v1;
        return true;
    }
    Eigen::Vector3d _p1,_p2;

};