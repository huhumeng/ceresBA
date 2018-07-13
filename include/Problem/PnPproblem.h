#pragma once

#include <ceres/ceres.h>
#include <vector>

#include "CostFunctions.h"

class PnPproblem{
public:

    PnPproblem(std::vector<Eigen::Vector3d>& p_ref, 
               std::vector<Eigen::Vector2d>& u_cur,
               double* initR, double* initT){

        for(size_t i=0; i<p_ref.size(); ++i)
        {   
        
            ceres::CostFunction* costfunction = 
                new ceres::AutoDiffCostFunction<Project3DTo2DPoseOnlyCost, 2, 3, 3>(
                    new Project3DTo2DPoseOnlyCost(p_ref[i], u_cur[i], fx, fy, cx, cy));

            problem.AddResidualBlock(costfunction, NULL, initR, initT);
        }
    }

    void solve(ceres::Solver::Options& option, ceres::Solver::Summary* summary){
        ceres::Solve(option, &problem, summary);
    }
private:
    // 520.9, 521.0, 325.1, 249.7
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    ceres::Problem problem;

};

class PnPPointproblem{
public:

    PnPPointproblem(std::vector<Eigen::Vector2d>& measure,
                    std::vector<Eigen::Vector3d>& pworld,
                    double* initR, double* initT){

        for(size_t i=0; i<measure.size(); ++i)
        {   
        
            ceres::CostFunction* costfunction = 
                new ceres::AutoDiffCostFunction<Project3DTo2DCost, 2, 3, 3, 3>(
                    new Project3DTo2DCost(measure[i], fx, fy, cx, cy));
            // double* p_pworld = ;
            problem.AddResidualBlock(costfunction, NULL, initR, initT, pworld[i].data());
        
        }
    }

    void solve(ceres::Solver::Options& option, ceres::Solver::Summary* summary){
        ceres::Solve(option, &problem, summary);
    }
private:
    // 520.9, 521.0, 325.1, 249.7
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    ceres::Problem problem;

};