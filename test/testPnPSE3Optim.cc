#include "PnPproblem.h"
#include "GenData.h"

#include <iostream>

using namespace std;

int main(){

    double r[3] = {1.0, 2.0, 3.0};
    double t[3] = {3.0, 2.0, 1.0};

    GenPointWithPoseKnown gener(r, t, 180);

    double initPose[6] = {1, 1, 1, 1, 1, 1};
    // double initT[3] = {};

    PnPPointproblemSE3 problem(gener.p2ds, gener.p3dnoise, initPose);

ceres::Solver::Options option;
    
    option.linear_solver_type = ceres::DENSE_SCHUR;
    
    option.minimizer_progress_to_stdout = true;
    
    option.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    
    problem.solve(option, &summary);
    
    cout << summary.BriefReport() << endl;
    
    cout << "Result is: " << endl;
    cout << initPose[0] << ", " << initPose[1] << ", " << initPose[2] << endl;
    cout << initPose[3] << ", " << initPose[4] << ", " << initPose[5] << endl;

    return 0;
}