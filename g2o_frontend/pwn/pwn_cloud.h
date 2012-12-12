#ifndef PWN_CLOUD_H
#define PWN_CLOUD_H

#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include "pwn_defs.h"

#define HI(num) (((num) & 0x0000FF00) >> 8)
#define LO(num) ((num) & 0x000000FF)

typedef std::vector<Vector6f,Eigen::aligned_allocator<Vector6f> > Vector6fVector;
typedef std::vector<Matrix6f,Eigen::aligned_allocator<Matrix6f> > Matrix6fVector;

typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> MatrixXus;

void cloud2mat(Vector6fPtrMatrix& pointsMat,
	       Matrix6fPtrMatrix& informationMat,
	       Vector6fVector& points,
	       Matrix6fVector& omegas,
	       const Eigen::Isometry3f& X,
	       const Eigen::Matrix3f& cameraMatrix,
	       Eigen::MatrixXf& zBuffer /* temp zBuffer for working. Allocate it outside */);

void cloud2mat(Vector6fPtrMatrix& pointsMat,
               Vector6fVector& points,
               const Eigen::Isometry3f& X,
               const Eigen::Matrix3f& cameraMatrix,
               Eigen::MatrixXf& zBuffer /* temp zBuffer for working. Allocate it outside*/);

void depth2cloud(Vector6fVector& cloud, const Eigen::MatrixXf& depth, const Eigen::Matrix3f& cameraMatrix);

void cloud2depth(Eigen::MatrixXf& depth, const Vector6fVector& cloud, const Eigen::Matrix3f& cameraMatrix);

void depth2img(MatrixXus& img, const Eigen::MatrixXf& depth);

void img2depth(Eigen::MatrixXf& depth, const MatrixXus& img);

bool readPgm(MatrixXus& img, FILE *pgmFile);

bool writePgm(const MatrixXus& img, FILE *pgmFile);

void skipComments(FILE *fp);

#endif
