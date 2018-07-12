#pragma once

#include <ceres/ceres.h>
#include <vector>

#include "CostFunctions.h"

class ICPproblem{
public:

    ICPproblem(std::vector<Eigen::Vector3d>& p_ref, 
               std::vector<Eigen::Vector3d>& p_cur,
               double* initR, double* initT){

        for(size_t i=0; i<p_ref.size(); ++i)
        {   
        
            ceres::CostFunction* costfunction = 
                new ceres::AutoDiffCostFunction<Project3DTo3DPoseOnlyCost, 3, 3, 3>(
                    new Project3DTo3DPoseOnlyCost(p_ref[i], p_cur[i]));

            problem.AddResidualBlock(costfunction, NULL, initR, initT);
        }
    }

    void solve(ceres::Solver::Options& option, ceres::Solver::Summary* summary){
        ceres::Solve(option, &problem, summary);
    }
private:

    ceres::Problem problem;

};