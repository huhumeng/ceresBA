#include "SE3.h"
#include <ceres/ceres.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
/// PoseBlockSize can only be
/// 7 (quaternion + translation vector) or
/// 6 (rotation vector + translation vector)
class ReprojectionErrorSE3XYZ7 : public ceres::SizedCostFunction<2, 7, 3>
{
public:
    ReprojectionErrorSE3XYZ7(double fx_,
                             double fy_,
                             double cx_,
                             double cy_,
                             double observation_x,
                             double observation_y)
        : fx(fx_), fy(fy_), cx(cx_), cy(cy_),
          _observation_x(observation_x),
          _observation_y(observation_y){
            //   std::cout << "Here is a construct function.\n";
          }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const{
        
        // std::cout << "Here is a cost function.\n";
        // parameters二维数组
        // parameters[0][0-3] 四元数
        // parameters[0][4-6] 平移
        // parameters[1][0-2] 3D点
        Eigen::Map<const Eigen::Quaterniond> quaterd(parameters[0]);
        Eigen::Map<const Eigen::Vector3d> trans(parameters[0] + 4);
        Eigen::Map<const Eigen::Vector3d> point(parameters[1]);

        // T_cw
        Eigen::Vector3d p = quaterd * point + trans;
        // std::cout << quaterd.coeffs() << std::endl;
        // std::cout << p << std::endl;
        // 残差定义
        // double f_by_z = f / p[2];
        residuals[0] = fx / p[2] * p[0] + cx - _observation_x;
        residuals[1] = fy / p[2] * p[1] + cy - _observation_y;

        // std::cout << residuals[0] << " " << residuals[1] << std::endl;

        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_cam;
        
        // double f_by_zz = f_by_z / p[2];
        J_cam << fx / p[2] , 0, - fx / (p[2] * p[2]) * p[0],
                 0, fy / p[2],  - fy / (p[2] * p[2]) * p[1];


        if(jacobians != NULL)
        {
            if(jacobians[0] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
                J_se3.setZero();
                J_se3.block<2,3>(0,0) = - J_cam * skew(p);
                J_se3.block<2,3>(0,3) = J_cam;
            }
            if(jacobians[1] != NULL)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > J_point(jacobians[1]);
                J_point = J_cam * quaterd.toRotationMatrix();
            }
        }

        return true;
    }

    double fx;
    double fy;
    double cx;
    double cy;

private:
    double _observation_x;
    double _observation_y;
};

class ReprojectionErrorSE3XYZ6 : public ceres::SizedCostFunction<2, 6, 3>
{
public:
    ReprojectionErrorSE3XYZ6(double fx_,
                             double fy_,
                             double cx_,
                             double cy_,
                             double observation_x,
                             double observation_y)
        : fx(fx_), fy(fy_), cx(cx_), cy(cy_),
          _observation_x(observation_x),
          _observation_y(observation_y){}

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const{

        Eigen::Quaterniond quaterd = toQuaterniond(Eigen::Map<const Eigen::Vector3d>(parameters[0]));
        Eigen::Map<const Eigen::Vector3d> trans(parameters[0] + 3);
        Eigen::Map<const Eigen::Vector3d> point(parameters[1]);
        
        // std::cout << "point : " << point << std::endl;
        // std::cout << "Quaterd : " << quaterd.coeffs() << std::endl;
        // std::cout << "Transla : " << trans << std::endl;
        Eigen::Vector3d p = quaterd * point + trans;
        // std::cout << "p : " << p << std::endl;
        // double f_by_z = f / p[2];
        residuals[0] = fx / p[2] * p[0] + cx - _observation_x;
        residuals[1] = fy / p[2] * p[1] + cy - _observation_y;
        // std::cout << "residuals : " << residuals[0] << " " << residuals[1] << std::endl;
        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_cam;
        // double f_by_zz = f_by_z / p[2];
        J_cam << fx / p[2], 0, - fx / (p[2] * p[2]) * p[0],
                 0, fy / p[2], - fy / (p[2] * p[2]) * p[1];

        if(jacobians != nullptr)
        {
            if(jacobians[0] != nullptr)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J_se3(jacobians[0]);
                J_se3.block<2,3>(0,0) = - J_cam * skew(p);
                J_se3.block<2,3>(0,3) = J_cam;
            }
            if(jacobians[1] != nullptr)
            {
                Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor>> J_point(jacobians[1]);
                J_point = J_cam * quaterd.toRotationMatrix();
            }
        }

        return true;
    }

    double fx;
    double fy;
    double cx;
    double cy;

private:
    double _observation_x;
    double _observation_y;
};

/// PoseBlockSize can only be
/// 7 (quaternion + translation vector) or
/// 6 (rotation vector + translation vector)
class PoseSE3Parameterization7 : public ceres::LocalParameterization {
public:
    PoseSE3Parameterization7() {}
    virtual ~PoseSE3Parameterization7(){}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const{
        
        Eigen::Map<const Eigen::Vector3d> trans(x + 4);
        SE3 se3_delta = SE3::exp(Eigen::Map<const Vector6d>(delta));

        Eigen::Map<const Eigen::Quaterniond> quaterd(x);
        Eigen::Map<Eigen::Quaterniond> quaterd_plus(x_plus_delta);
        Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 4);

        quaterd_plus = se3_delta.rotation() * quaterd;
        trans_plus = se3_delta.rotation() * trans + se3_delta.translation();

        return true;
    }

    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const{
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor> > J(jacobian);
        J.setZero();
        J.block<6,6>(0, 0).setIdentity();
        return true;
    }
    virtual int GlobalSize() const { return 7; }
    virtual int LocalSize() const { return 6; }
};

class PoseSE3Parameterization6 : public ceres::LocalParameterization {
public:
    PoseSE3Parameterization6() {}
    virtual ~PoseSE3Parameterization6(){}

    // LinearOPlus
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const{
        
        Eigen::Map<const Eigen::Vector3d> trans(x + 3);

        // exp(delta^) from se3 to SE3
        SE3 se3_delta = SE3::exp(Eigen::Map<const Vector6d>(delta));

        // 新的Rotation
        Eigen::Quaterniond quaterd_plus = se3_delta.rotation() * toQuaterniond(Eigen::Map<const Eigen::Vector3d>(x));
        
        // 结果
        Eigen::Map<Eigen::Vector3d> angles_plus(x_plus_delta);
        angles_plus = toAngleAxis(quaterd_plus);

        Eigen::Map<Eigen::Vector3d> trans_plus(x_plus_delta + 3);
        trans_plus = se3_delta.rotation() * trans + se3_delta.translation();
        return true;
    }

    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const{
        Eigen::Map<Eigen::Matrix<double, 6, 6, Eigen::RowMajor> > J(jacobian);
        J.setIdentity();
        return true;
    }
    virtual int GlobalSize() const { return 6; }
    virtual int LocalSize() const { return 6; }
};

