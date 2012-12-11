#ifndef PWN_DEFS_H
#define PWN_DEFS_H

#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Matrix<float, 6, 1> Vector6f;
typedef Eigen::Matrix<float, 6, 6> Matrix6f;

typedef Eigen::Matrix<Vector6f*, Eigen::Dynamic, Eigen::Dynamic> Vector6fPtrMatrix;
typedef Eigen::Matrix<Matrix6f*, Eigen::Dynamic, Eigen::Dynamic> Matrix6fPtrMatrix;

#endif
