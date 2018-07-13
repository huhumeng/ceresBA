#include <iostream>
#include <ceres/ceres.h>

#include "GenData.h"
#include "ICPproblem.h"

using namespace std;
int main(){

    double r[3] = {1.0, 2.0, 3.0};
    double t[3] = {3.0, 2.0, 1.0};

    GenPointWithPoseKnown gener(r, t, 50);

    double initR[3] = {0.0, 0.0, 1.0};
    double initT[3] = {0.0, 0.0, 0.0};
    
    ICPproblem problem(gener.p3ds, gener.pc3ds, initR, initT);

    ceres::Solver::Options option;
    
    option.linear_solver_type = ceres::DENSE_SCHUR;
    
    option.minimizer_progress_to_stdout = true;
    
    ceres::Solver::Summary summary;
    
    problem.solve(option, &summary);
    
    cout << summary.BriefReport() << endl;
    
    cout << "Result is: " << endl;
    cout << initR[0] << ", " << initR[1] << ", " << initR[2] << endl;
    cout << initT[0] << ", " << initT[1] << ", " << initT[2] << endl;

    return 0;
}