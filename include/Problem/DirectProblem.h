#include "DirectCost.h"
#include "CostFunctionsSE3.h"

#include <opencv2/core/core.hpp>

class DirectPoseProblem{
public:

    DirectPoseProblem( const std::vector<double>& measure,
                       Eigen::Vector3d& point,
                       cv::Mat* image,
                       double* initPose){
        
        problem.AddParameterBlock(initPose, 6, new PoseSE3Parameterization6);
        
        for(size_t i=0; i<measure.size(); ++i)
        {   
        
            // problem.AddParameterBlock(pworld[i].data(), 3);
            // ordering->AddElementToGroup(pworld[i].data(), 0);

            ceres::CostFunction* costFunc = 
                new PhotometricErrorSE3Only6(point, fx, fx, cx, cy, measure[i], image);
            
            // ceres::CostFunction* costFunc = 
            //     new ReprojectionErrorSE3XYZ7(fx, fy, cx, cy, measure[i][0], measure[i][1]);

            problem.AddResidualBlock(costFunc, nullptr, initPose);
            // problem.SetParameterBlockConstant(pworld[i].data());
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