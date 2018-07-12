#include "PnPproblem.h"

void PnPproblem::solve(){
    
    // ceres::Problem problem;
//     for(int i=0;i<image1_3d.size();i++)
//     {
//         ceres::CostFunction* costfunction=new ceres::AutoDiffCostFunction<cost_function_define,2,3,3>(new cost_function_define(image2_3d[i],image1_3d[i]));
//         problem.AddResidualBlock(costfunction,NULL,cere_r,cere_t);
//     }
//   ceres::Solver::Options option;
//   option.linear_solver_type=ceres::DENSE_SCHUR;
//   //输出迭代信息到屏幕
//   option.minimizer_progress_to_stdout=true;
//   //显示优化信息
//   ceres::Solver::Summary summary;
//   //开始求解
//   ceres::Solve(option,&problem,&summary);
//   //显示优化信息
//   cout<<summary.BriefReport()<<endl;
}