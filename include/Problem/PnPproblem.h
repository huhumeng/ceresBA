#pragma once

#include <ceres/ceres.h>
#include <vector>

#include "CostFunctions.h"
#include "CostFunctionsSE3.h"

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

class PnPPointproblemSE3{
public:

    PnPPointproblemSE3(const std::vector<Eigen::Vector2d>& measure,
                       std::vector<Eigen::Vector3d>& pworld,
                       double* initPose){
        loss_function = new ceres::CauchyLoss(1.0);
        
        // ordering = new ceres::ParameterBlockOrdering;
        // problem.AddParameterBlock(initPose, 7, new PoseSE3Parameterization7);
        problem.AddParameterBlock(initPose, 6, new PoseSE3Parameterization6);
        // ordering->AddElementToGroup(initPose, 1);
        // std::cout << "Here is a pnp construct function." << measure.size() << std::endl;
        for(size_t i=0; i<measure.size(); ++i)
        {   
        
            problem.AddParameterBlock(pworld[i].data(), 3);
            // ordering->AddElementToGroup(pworld[i].data(), 0);

            ceres::CostFunction* costFunc = 
                new ReprojectionErrorSE3XYZ6(fx, fy, cx, cy, measure[i][0], measure[i][1]);
            
            // ceres::CostFunction* costFunc = 
            //     new ReprojectionErrorSE3XYZ7(fx, fy, cx, cy, measure[i][0], measure[i][1]);

            problem.AddResidualBlock(costFunc, nullptr, initPose, pworld[i].data());
            problem.SetParameterBlockConstant(pworld[i].data());
        }
    }

    void solve(ceres::Solver::Options& option, ceres::Solver::Summary* summary){
        ceres::Solve(option, &problem, summary);
    }
private:
    // 520.9, 521.0, 325.1, 249.7
    double fx = 520.9, fy = 521.0, cx = 325.1, cy = 249.7;
    ceres::Problem problem;
    ceres::ParameterBlockOrdering* ordering = nullptr;
    ceres::LossFunction *loss_function = nullptr;

};