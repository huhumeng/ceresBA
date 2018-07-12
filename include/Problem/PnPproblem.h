#pragma once

#include <ceres/ceres.h>

class PnPproblem{
public:

    void solve();   

private:

    ceres::Problem problem;
    
};

