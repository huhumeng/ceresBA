#include <vector>
#include "ceres/ceres.h"
#include "glog/logging.h"

using ceres::CostFunction;
using ceres::SizedCostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

using namespace std;

// class inheritance of SizedCostFunction
class QuadraticCostFunction
  : public SizedCostFunction<1 /* number of residuals */,
                             1 /* size of first parameter */> {
 public:
  virtual ~QuadraticCostFunction() {}

  virtual bool Evaluate(double const* const* parameters,
                        double* residuals,
                        double** jacobians) const {
    double x = parameters[0][0];

    // 残差函数
    residuals[0] = 10 - x;
    cout << residuals[0] << endl;
    // f'(x) = -1. 只有一个参数，参数只有一个维度，jacobians只有一个元素
    //
    // 因为jacobians可以设置为NULL, Evaluate必须检查jacobians是否需要计算
    //
    // 对这个简单问题，检查jacobians[0]是否为空是足够的，但是通常的CostFunctions来说，
    // Ceres可能只需要参数的子集的导数
    if (jacobians != NULL && jacobians[0] != NULL) {
      jacobians[0][0] = -1;
    }
    return true;
  }
};
// main 
int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);

  // 变量初始值
  double x = 0.5;
  const double initial_x = x;

  // Build the problem.
  Problem problem;

  // 构建CostFunction（残差方程）
  CostFunction* cost_function = new QuadraticCostFunction;
  // CostFunction为QuadraticCostFunction，不使用LossFunction,只有一个优化参数x
  problem.AddResidualBlock(cost_function, NULL, &x);

  // Run the solver!
  Solver::Options options;
  options.minimizer_progress_to_stdout = false;
  Solver::Summary summary;
  Solve(options, &problem, &summary);

//   std::cout << summary.BriefReport() << "\n";
  std::cout << "x : " << initial_x
            << " -> " << x << "\n";

  return 0;
}