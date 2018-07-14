#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <opencv2/core/core.hpp>
#include <Eigen/Core>

#include "GenData.h"
#include "SE3.h"

// This cannot work
// Auto Difference cannot differ different level error.
// i.e. Some error do none contribution to the optimization,
// so their jacobian should set to zero, but in auto difference api,
// this is not implement with my knowledge. If you find some better
// solution, please share with me.
struct Direct3DToUVPoseOnly{

public:
    Direct3DToUVPoseOnly(Eigen::Vector3d& _point, 
                     double _fx, double _fy, 
                     double _cx, double _cy,
                     double _measurement,
                     cv::Mat& _image)
                     :point(_point), 
                     fx(_fx), fy(_fy), cx(_cx), cy(_cy),
                     image(_image),
                     measurement(_measurement){
                         camera = new CameraModel(fx, fy, cx, cy, 640.0, 480.0);
                     }
    ~Direct3DToUVPoseOnly(){
        delete camera;
    }
    
    template <class T>
    bool operator()(const T* const rotation, 
                    const T* const translat,
                    T* residuals) const{
        
        T p_c[3];
        ceres::AngleAxisRotatePoint(rotation, point.data(), p_c);
        const T& x = p_c[0] * fx / p_c[2] + cx;
        const T& y = p_c[1] * fy / p_c[2] + cy;

        // Eigen::Map<Eigen::Vector2d> err(residuals);

        if(camera->isInCamera(x, y)){
            residuals = getPixelValue(x, y) - measurement;
        }else{
            residuals = 0;
        }
            
    }

private:

    inline int getPixelValue(double x, double y);

    // 当前的灰度值
    const double& measurement;

    Eigen::Vector3d point;
    
    float fx, fy, cx, cy;

    cv::Mat& image;

    CameraModel* camera;
};

class PhotometricErrorSE3Only6 : public ceres::SizedCostFunction<1, 6>{

public:
    PhotometricErrorSE3Only6(Eigen::Vector3d& _point, 
                             double _fx, double _fy, 
                             double _cx, double _cy,
                             double _measurement,
                             cv::Mat* _image)
                             :point(_point), 
                             fx(_fx), fy(_fy), cx(_cx), cy(_cy),
                             image(_image),
                             measurement(_measurement){
        camera = new CameraModel(fx, fy, cx, cy, 640.0, 480.0);
    }
    ~PhotometricErrorSE3Only6(){
        delete camera;
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const{
        // Quaternion
        Eigen::Quaterniond quaterd = toQuaterniond(Eigen::Map<const Eigen::Vector3d>(parameters[0]));
        
        // Translation
        Eigen::Map<const Eigen::Vector3d> trans(parameters[0] + 3);

        Eigen::Vector3d point_c = quaterd * point + trans;

        double x = fx * point_c[0] / point_c[2] + cx;
        double y = fy * point_c[1] / point_c[2] + cy;

        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_cam;
        // double f_by_zz = f_by_z / p[2];
        J_cam << fx / point_c[2], 0, - fx / (point_c[2] * point_c[2]) * point_c[0],
                 0, fy / point_c[2], - fy / (point_c[2] * point_c[2]) * point_c[1];
        
        if(camera->isVisable(x, y)){
            residuals[0] = (double)getPixelValue(x, y) - measurement;
            if(jacobians!=nullptr){
                if(jacobians[0]!=nullptr){
                    Eigen::Map<Eigen::Matrix<double, 1, 6>> Jacobian_Xi(jacobians[0]);
                    Eigen::Matrix<double, 1, 2> J_pixel_uv;
                    J_pixel_uv(0, 0) = ((double)getPixelValue(x+1, y) - getPixelValue(x-1, y)) / 2;
                    J_pixel_uv(0, 1) = ((double)getPixelValue(x, y+1) - getPixelValue(x, y-1)) / 2;
                    Eigen::Matrix<double, 2, 6> J_uv_se3;
                    J_uv_se3.block<2,3>(0,0) = - J_cam * skew(point_c);
                    J_uv_se3.block<2,3>(0,3) = J_cam;
                    
                    Jacobian_Xi = J_pixel_uv * J_uv_se3;
                }
            }
        }else{
            residuals[0] = 0.0;
            if(jacobians!=nullptr){
                if(jacobians[0]!=nullptr){
                    Eigen::Map<Eigen::Matrix<double, 1, 6>> Jacobian_Xi(jacobians[0]);
                    Jacobian_Xi.setZero();
                    
                }
            }
        }
        return true;
    }

private:

    inline int getPixelValue(double x, double y) const;

    // 当前的灰度值
    double measurement;

    Eigen::Vector3d point;
    
    float fx, fy, cx, cy;

    cv::Mat* image;

    CameraModel* camera;

};

// Implement Direct Method refer to GaoXiang's slambook 
// which uses g2o as an nonlinear optimization framework.
class PhotometricErrorSE3Depth6 : public ceres::SizedCostFunction<1, 6, 1>{
public:
    PhotometricErrorSE3Depth6(Eigen::Vector2d& _point, 
                            double _fx, double _fy, 
                            double _cx, double _cy,
                            double _measurement,
                            cv::Mat& _image)
                            :point(_point), 
                            fx(_fx), fy(_fy), cx(_cx), cy(_cy),
                            image(_image),
                            measurement(_measurement)
    {
        camera = new CameraModel(fx, fy, cx, cy, 640.0, 480.0);
    }
    ~PhotometricErrorSE3Depth6(){
        delete camera;
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const{

        // Quaternion
        Eigen::Quaterniond quaterd = toQuaterniond(Eigen::Map<const Eigen::Vector3d>(parameters[0]));
        
        // Translation
        Eigen::Map<const Eigen::Vector3d> trans(parameters[0] + 3);
        
        // inverse depth value
        const double& invdepth = parameters[1][0];
        double xyz[3];
        xyz[0] = (point[0] - cx) / (invdepth * fx);
        xyz[1] = (point[1] - cy) / (invdepth * fy);
        xyz[2] = 1 / (invdepth);

        Eigen::Map<Eigen::Vector3d> point_w(xyz);
        Eigen::Vector3d point_c = quaterd * point_w;
        point_c += trans;

        double u = fx / point_c[2] * point_c[0] + cx;
        double v = fy / point_c[2] * point_c[1] + cy;

        Eigen::Matrix<double, 2, 3, Eigen::RowMajor> J_cam;
        // double f_by_zz = f_by_z / p[2];
        J_cam << fx / point_c[2], 0, - fx / (point_c[2] * point_c[2]) * point_c[0],
                 0, fy / point_c[2], - fy / (point_c[2] * point_c[2]) * point_c[1];

        if(camera->isInCamera(u, v))
            residuals[0] = (double)getPixelValue(u, v) - measurement;
        else{
            if(jacobians != NULL){
                if(jacobians[0] != NULL){
                    Eigen::Map<Eigen::Matrix<double, 2, 6, Eigen::RowMajor>> J_se3(jacobians[0]);
                    J_se3.block<2,3>(0,0) = - J_cam * skew(point_c);
                    J_se3.block<2,3>(0,3) = J_cam;
                }

            }
        }
        if(jacobians != NULL)
        {
            if(jacobians[0] != NULL)
            {
                // Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>> J_se3(jacobians[0]);
                // J_se3.setZero();
                // J_se3.block<2,3>(0,0) = - J_cam * skew(p);
                // J_se3.block<2,3>(0,3) = J_cam;
            }
            if(jacobians[1] != NULL)
            {
                // Eigen::Map<Eigen::Matrix<double, 2, 3, Eigen::RowMajor> > J_point(jacobians[1]);
                // J_point = J_cam * quaterd.toRotationMatrix();
            }
        }

        return true;
    }

private:

    inline int getPixelValue(double x, double y) const;

    // grayscale in ref frame
    const double& measurement;

    // point in ref_frame
    Eigen::Vector2d point;
    
    float fx, fy, cx, cy;

    cv::Mat& image;

    CameraModel* camera;
};