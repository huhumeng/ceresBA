#include "PnPproblem.h"
#include "GenData.h"
#include "SE3.h"

#include <iostream>

using namespace std;
#define USE_SIX_PARA 1

int main(){

    double r[3] = {1.0, 2.0, 3.0};
    double t[3] = {3.0, 2.0, 1.0};

    Eigen::Map<const Eigen::Vector3d> rotation(r);

    double norm = rotation.norm();

    Eigen::AngleAxisd rotation_vector(norm, rotation/norm);

    Eigen::Quaterniond q(rotation_vector);
    cout << q.toRotationMatrix() << endl;
    // cout << q.coeffs() << endl;

    GenPointWithPoseKnown gener(r, t, 400);

#if USE_SIX_PARA
    double initPose[6] = {1.1, 1.1, 1.1, 2.0, 1.0, 1.0};
    // cout << gener.p3ds[0] << endl;
    PnPPointproblemSE3 problem(gener.p2ds, gener.p3ds, initPose);
    
#else
    //  0.255322
    //  0.510644
    //  0.765966
    // -0.295551
    double initPose[7] = {0.29551, -0.255322, -0.510644, -0.765966, 3.0, 2.0, 1.0};
    PnPPointproblemSE3 problem(gener.p2ds, gener.p3ds, initPose);
#endif
    ceres::Solver::Options option;
    
    option.linear_solver_type = ceres::SPARSE_SCHUR;
    option.trust_region_strategy_type = ceres::DOGLEG;
    option.minimizer_progress_to_stdout = false;
    
    option.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    
    problem.solve(option, &summary);
    
    cout << summary.BriefReport() << endl;
    
    cout << "Result is: " << endl;
#if USE_SIX_PARA
    cout << initPose[0] << ", " << initPose[1] << ", " << initPose[2] << endl;
    cout << initPose[3] << ", " << initPose[4] << ", " << initPose[5] << endl;
    // cout << gener.pc3ds[0] << endl;
#else
    Eigen::Quaterniond q(initPose[3], initPose[0], initPose[1], initPose[2]);
    cout << toAngleAxis(q, nullptr) << endl;
    // cout << initPose[0] << ", " << initPose[1] << ", " << initPose[2] << endl;
    cout << initPose[4] << ", " << initPose[5] << ", " << initPose[6] << endl;
#endif

    Eigen::Vector3d ro(-0.679252, -1.3585, -2.03776);
    cout << toQuaterniond(ro).toRotationMatrix() << endl;
    return 0;
}