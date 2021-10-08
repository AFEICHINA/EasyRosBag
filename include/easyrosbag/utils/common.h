#ifndef _COMMON_H_
#define _COMMON_H_

#include <Eigen/Core>
#include <Eigen/Dense>

namespace easyrosbag{
    struct ImuOutput
    {
        /* data */
        double timestamp = 0;
        Eigen::Vector3d acc = Eigen::Vector3d(0,0,0);
        Eigen::Vector3d gyr = Eigen::Vector3d(0,0,0);;
    };

    struct Pose
    {
        double timestamp = 0;
        Eigen::Vector3d pose = Eigen::Vector3d(0,0,0);
        Eigen::Quaterniond quat = Eigen::Quaterniond(1,0,0,0);
    };

    struct Path
    {
        double timestamp = 0;
        Eigen::Vector3d pose = Eigen::Vector3d(0,0,0);
    };
}

#endif //_COMMON_H_