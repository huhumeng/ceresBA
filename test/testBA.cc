#include "generateData.h"

#include <iostream>
#include <Eigen/Geometry>
#include "ceresBA.h"
#include <glog/logging.h>

using namespace std;
int main(){

    // double d = gaussion(0, 1);

    // for(int i=0; i<100; i++){
    //     cout << uniform(0, 100) << endl;
    //     cout << uniform(0.0, 100.0) << endl;
    // }

    double r[3] = {1.0, 2.0, 3.0};
    double t[3] = {3.0, 2.0, 1.0};

    generatePointAndPose gener;
    gener.setPose(r, t);
    gener.genWorldPoint();
    gener.genCameraPoint();

    double initR[3] = {0.0, 0.0, 1.0};
    double initT[3] = {0.0, 0.0, 0.0};
    ceres::Problem problem;
    for(int i=0;i<gener.p3ds.size();++i)
    {
        ceres::CostFunction* costfunction = 
            new ceres::AutoDiffCostFunction<Project3DTo3DPoseOnlyCost, 2, 3, 3>(
                new Project3DTo3DPoseOnlyCost(gener.p3ds[i], gener.pc3ds[i]));

        problem.AddResidualBlock(costfunction, NULL, initR, initT);
    }
    
    ceres::Solver::Options option;
    option.linear_solver_type=ceres::DENSE_SCHUR;
    //输出迭代信息到屏幕
    option.minimizer_progress_to_stdout=true;
    //显示优化信息
    ceres::Solver::Summary summary;
    //开始求解
    ceres::Solve(option,&problem,&summary);
    //显示优化信息
    cout<<summary.BriefReport()<<endl;
    // Eigen::Vector3d rotation(r);
    // Eigen::Vector3d translat(t);

    // // cout << rotation.norm() << endl;

    // // r[0] = 2.0;
    // // r[1] = 2.0;

    // // cout << rotation << endl;
    // double theta = rotation.norm();
    // Eigen::AngleAxisd roation_vector(theta, rotation/theta);
    // Eigen::Quaterniond q(roation_vector);

    // for(size_t i=0; i<gener.p3ds.size(); ++i){
    //     cout << "Pc 3D show be:\n" << q*gener.p3ds[i] + translat  - gener.pc3ds[i] << endl;
    // }


    return 0;
}