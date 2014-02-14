#ifndef UTILITY_H
#define UTILITY_H


#include <Eigen/Core>
#include <Eigen/Geometry>



namespace utility
{
    //Convert an isometry matrix into an equivalent vector
    Eigen::Vector3d t2v(const Eigen::Isometry2d& iso);
    //Convert a vector into an equivalent isometry matrix
    Eigen::Isometry2d v2t(const Eigen::Vector3d& v);
}

#endif //UTILITY_H
