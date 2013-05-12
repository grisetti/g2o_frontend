#include <iostream>
#include <fstream>
#include <string>
#include <set>
#include <sstream>

#include "scene_merger.h"


using namespace Eigen;
using namespace g2o;
using namespace std;

  SceneMerger::SceneMerger(){
    _distanceThreshold = 0.1;
    _normalThreshold = cos(20*M_PI/180.0f);
    _normalGenerator = 0;
    _indexImage.resize(480,640);
    _zBuffer.resize(480,540);
    _maxPointDepth = 10;
    // _cameraMatrix << 
    //   525.0f, 0.0f, 319.5f,
    //   0.0f, 525.0f, 239.5f,
    //   0.0f, 0.0f, 1.0f;
    _cameraMatrix << 
    570.342f, 0.0f, 319.5f,
    0.0f, 570.342f, 239.5f,
    0.0f, 0.0f, 1.0f;


  }
    

  void SceneMerger::merge(Scene* scene, Eigen::Isometry3f& transform){
    assert (_normalGenerator && "you need to set the normal generator");
    scene->_points.toIndexImage(_indexImage, _zBuffer, _cameraMatrix, transform, _maxPointDepth);

    // scan all the points, 
    // if they fall in a cell not with -1, 
    //   skip
    // if they fall in a cell with n>1, 
    //   if distance is incompatible,
    //      skip
    // if notrmals are incompatible
    //      skip
    // accumulate the point in the cell i
    // set the target accumulator  to i;
    PixelMapper pixelMapper;
    pixelMapper.setCameraMatrix(_cameraMatrix);
    pixelMapper.setTransform(transform);

    _collapsedIndices.resize(scene->points().size());
    std::fill(_collapsedIndices.begin(), _collapsedIndices.end(), -1);
    
    int killed=0;
    int currentIndex=0;
    for (PointWithNormalVector::const_iterator it = scene->_points.begin(); it!=scene->_points.end(); currentIndex++ ,it++){
      const PointWithNormal& currentPoint = *it;
      Vector3f ip=pixelMapper.projectPoint(currentPoint.head<3>());
      Vector2i coords=pixelMapper.imageCoords(ip);
      if (ip.z()<0 || ip.z() > _maxPointDepth || 
	  coords.x()<0 || coords.x()>=_zBuffer.cols() || 
	  coords.y()<0 || coords.y()>=_zBuffer.rows())
	continue;
      float& targetZ = _zBuffer(coords.y(), coords.x());
      int targetIndex =_indexImage(coords.y(), coords.x());
      const PointWithNormal& targetPoint = scene->points()[targetIndex];
      float currentZ=ip.z();
      if (targetIndex <0){
	continue;
      }
      if (targetIndex == currentIndex){
	_collapsedIndices[currentIndex] = currentIndex;
      } else if (fabs(currentZ-targetZ)<_distanceThreshold && currentPoint.normal().dot(targetPoint.normal()) > _normalThreshold){
	Gaussian3f& targetGaussian = scene->_gaussians[targetIndex];
	Gaussian3f& currentGaussian = scene->_gaussians[currentIndex];
	targetGaussian.addInformation(currentGaussian);
	_collapsedIndices[currentIndex]=targetIndex;
	killed ++;
      }
    }
    cerr << "killed: " << killed  << endl;
    // scan the vector of covariances.
    // if the index is -1
    //    copy into k
    //    increment k 
    // if the index is the same,
    //    update the point with normal
    //    copy into k
    //    increment k

    int murdered = 0;
    int k=0;
    for (size_t i=0; i<_collapsedIndices.size(); i++){
      int collapsedIndex = _collapsedIndices[i];
      if (collapsedIndex == (int)i){
	scene->_points[i].setPoint(scene->_gaussians[i].mean());
      }
      if (collapsedIndex <0 || collapsedIndex == (int)i){
	scene->_points[k] = scene->_points[i];
	scene->_svds[k] = scene->_svds[i];
	scene->_gaussians[k] = scene->_gaussians[i];
	k++;
      } else {
	murdered ++;
      }
    }
    
    int originalSize = scene->size();
    // kill the leftover points
    scene->_points.resize(k);
    scene->_gaussians.resize(k);
    scene->_svds.resize(k);
    cerr << "murdered: " << murdered  << endl;
    cerr << "resized: " << originalSize << "->" << k << endl;
    
    // recompute the normals
    scene->_points.toIndexImage(_indexImage, _zBuffer, _cameraMatrix, transform, 10);
    _normalGenerator->computeNormalsAndSVD(scene->_points, scene->_svds, _indexImage, _cameraMatrix, transform);
  }
