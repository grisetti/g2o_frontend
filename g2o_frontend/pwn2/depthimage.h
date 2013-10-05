/** \file depthimage.h
 *  \brief DepthImage class header file.
 *  This is the header file of the DepthImage class.
*/

#ifndef _PWN_DEPTH_IMAGE_H_
#define _PWN_DEPTH_IMAGE_H_
#include "g2o_frontend/boss_logger/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "g2o_frontend/boss/blob.h"

#include "opencv2/core/core.hpp"

namespace pwn {

/** \typedef MatrixXus
 *  \brief Unsigned short matrix type.
 *
 *  The MatrixXus type it's an Eigen matrix of unsigned short with generic dimensions.
*/
typedef Eigen::Matrix<unsigned short, Eigen::Dynamic, Eigen::Dynamic> MatrixXus;

/**
 *  Base class for input/output operations on depth images. 
 *  Depth images are basically matrices of unsigned short, this class extends the 
 *  Eigen MatrixXf class so that it is possible to do the basic input/output operations with
 *  depth images. The DepthImage object will contain the depth image values expressed in 
 *  metrs.
 */

class DepthImage: public Eigen::MatrixXf {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  /**
   *  Empty constructor.
   *  This constructor creates a DepthImage object with zero columns and zero rows. If the 
   *  optional input parameters are given and they are both different from zero the matrix 
   *  is filled with the maximum float representable.
   *  @param r is optional and can be used to define the number of columns of the matrix
   *  representing a depth image.
   *  @param c is optional and can be used to define the number of rows of the matrix
   *  representing a depth image.
   */
  DepthImage(int r = 0, int c = 0);
  
  /**
   *  Constructor using a matrix of unsigned short.
   *  This constructor creates a DepthImage object taking the values from a matrix of 
   *  unsigned short.
   *  @param m is a matrix of unsigned short containing the depth values expressed 
   *  in millimeters.
   */
  DepthImage(const MatrixXus &m, float scaleFactor = 0.001f);
  
  /**
   *  This method generates a matrix of unsigned short containing the values of the 
   *  DepthImage object expressed in millimeters. If the optional input parameter is given,
   *  then all the values above it will be pruned and setted to zero.
   *  @param m is a matrix of unsigned short where the output values are stored.
   *  @param dmax is optional and can be used to prune depth values above its value.
   */
  void toUnsignedShort(MatrixXus &m, float dmax = std::numeric_limits<float>::max(), float scaleFactor = 0.001f) const;
  
  /**
   *  This method updates the current depth values of the DepthImage object using the values
   *  inside the input unsigned short matrix.
   *  @param m is a matrix of unsigned short containing the depth values expressed 
   *  in millimeters.
   */
  void fromUnsignedShort(const MatrixXus &m, float scaleFactor = 0.001f);

  /**
   *  This method generates an image of unsigned short containing the values of the 
   *  DepthImage object expressed in millimeters. If the optional input parameter is given,
   *  then all the values above it will be pruned and setted to zero.
   *  @param m is cv mat (type CV_16UC1)US
   *  @param dmax is optional and can be used to prune depth values above its value.
   */
  void toCvMat(cv::Mat &m, float dmax = std::numeric_limits<float>::max()) const;
 
  /**
   *  This method updates the current depth values of the DepthImage object using the values
   *  inside the input unsigned short image.
   *  @param m is an image of unsigned short containing the depth values expressed 
   *  in millimeters.
   */
  void fromCvMat(const cv::Mat &m, float scaleFactor = 0.001f);
  void fromCvMat32FC1(const cv::Mat &m, float scaleFactor = 0.001f);
  
  /**
   *  This method laods the values of a depth image file in the DepthImage object. 
   *  The file must be a .pgm image file.
   *  @param filename is a pointer to a string containing the filename of a .pgm depth image
   *  to load.
   */
  bool load(const char* filename, bool transposed = false, float scaleFactor = 0.001f); 
  
  /**
   *  This method saves the current depth values of the DepthImage object to an image file.
   *  The output image is saved in .pgm format. 
   *  @param filename is a pointer to a string containing the filename of the .pgm depth 
   *  image to save.
   */
  bool save(const char* filename, bool transposed = false, float scaleFactor = 0.001f) const;


  /**
   *  This static methos scales an input image of a desired integer
   *  fator. To have an image that is 1/2 of the original, you should
   *  set the step at 2;
   *  @param dest: the destination image that will be scaled
   *  @param src: source image
   *  @param step: the inverse of the scaling factor
   */
  static void scale(Eigen::MatrixXf& dest, const Eigen::MatrixXf& src, int step);
};

}

#endif
