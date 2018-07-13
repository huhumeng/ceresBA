#include <iostream>
#include <ceres/ceres.h>

#include "GenData.h"
#include "PnPproblem.h"

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#define OPTIMIZE_ONLY_POSE 1
using namespace std;
int main(){

    double r[3] = {1.0, 2.0, 3.0};
    double t[3] = {3.0, 2.0, 1.0};

    GenPointWithPoseKnown gener(r, t, 480);

    double initR[3] = {10.1, 21.0, 11.0};
    double initT[3] = {100.0, 20.1, 31.1};

    // Eigen::Quaterniond q(1.0, 0, 0, 0);
    // cout << q.coeffs() << endl;

#if OPTIMIZE_ONLY_POSE   
    PnPproblem problem(gener.p3ds, gener.p2ds, initR, initT);
#else
    // cv::Mat K = cv::Mat_<double>(3, 3) << (520.9, 0.0, 521.0, 0, 325.1, 249.7, 0, 0, 1);
    
    // cv::solvePnP();
    PnPPointproblem problem(gener.p2ds, gener.p3dnoise, initR, initT);
#endif
    ceres::Solver::Options option;
    
    option.linear_solver_type = ceres::DENSE_SCHUR;
    option.trust_region_strategy_type = ceres::DOGLEG;
    option.minimizer_progress_to_stdout = true;
    
    option.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    
    problem.solve(option, &summary);
    
    cout << summary.BriefReport() << endl;
    
    cout << "Result is: " << endl;
    cout << initR[0] << ", " << initR[1] << ", " << initR[2] << endl;
    cout << initT[0] << ", " << initT[1] << ", " << initT[2] << endl;

    return 0;
}