#ifndef __BOX_H__
#define __BOX_H__

#include <Eigen/Geometry>

struct  BoxQ
{
    Eigen::Vector3f bboxTransform_;
    Eigen::Quaternionf bboxQuaternion_;
    float cube_length_;
    float cube_width_;
    float cube_height_;
};


struct  Box
{
    float x_min_;
    float y_min_;
    float z_min_;
    float x_max_;
    float y_max_;
    float z_max_;
};

#endif