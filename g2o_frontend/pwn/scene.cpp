#include "scene.h"
#include <iostream>
using namespace std;


void Scene::toIndexImage(Eigen::MatrixXi& indexImage, Eigen::MatrixXf& zBuffer, 
		    const Eigen::Matrix3f& cameraMatrix, const Eigen::Isometry3f& cameraPose,
			 float dmax){
  _points.toIndexImage(indexImage, zBuffer, cameraMatrix, cameraPose, dmax);
}

void Scene::transform(const Eigen::Isometry3f& T){
  assert(_points.size() == _svds.size());
  assert(_points.size() == _gaussians.size());
  for (size_t i=0; i<_points.size(); i++){
    _points[i] = T*_points[i];
    _gaussians[i] = T*_gaussians[i];
    _svds[i] = T*_svds[i];
  }
}

void Scene::add(const Scene& scene, const Eigen::Isometry3f& T){
  size_t k=size();
  _points.resize(k+scene.size());
  _svds.resize(k+scene.size());
  _gaussians.resize(k+scene.size());
  int i=0;
  for (; k<size(); k++, i++){
    _points[k] = T*scene._points[i];
    _svds[k] = T*scene._svds[i];
    _gaussians[k] = T*scene._gaussians[i];
  }
}

void Scene::clear(){
  _points.clear();
  _svds.clear();
  _gaussians.clear();
}

void Scene::_updatePointsFromGaussians(bool eraseNormals){
  _gaussians.toPointWithNormalVector(_points, eraseNormals);
  _svds.resize(_points.size());
}

void Scene::_updateSVDsFromPoints(PointWithNormalStatistcsGenerator & generator, 
				  const Eigen::Matrix3f& cameraMatrix, 
				  const Eigen::Isometry3f& cameraPose,
				  int r, int c, float dmax){
  // generates an index image of the predefined size;
  Eigen::MatrixXi indexImage(r,c);
  Eigen::MatrixXf zBuffer(r,c);
  _points.toIndexImage(indexImage, zBuffer, cameraMatrix, cameraPose, dmax);
  // assumes the gaussians and the points are consistent
  generator.computeNormalsAndSVD(_points, _svds, indexImage, cameraMatrix, cameraPose);
}

DepthFrame::DepthFrame() {
  _baseline = 0.075;
  _cameraMatrix <<   
    525.0f, 0.0f, 319.5f,
    0.0f, 525.0f, 239.5f,
    0.0f, 0.0f, 1.0f;
  _maxDistance=10;
}

void DepthFrame::setImage(const DepthImage& image_){
  _image = image_;
  _updateGaussiansFromImage();
  _updatePointsFromGaussians();
}


void DepthFrame::_updateGaussiansFromImage(){
  _gaussians.fromDepthImage(_image, _cameraMatrix, _maxDistance, _baseline, 0.1);
}
