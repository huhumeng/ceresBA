#include "ceres/ceres.h"
#include "ceres/rotation.h"

#include <opencv2/core/core.hpp>
#include <Eigen/Core>

#include "GenData.h"

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

// 