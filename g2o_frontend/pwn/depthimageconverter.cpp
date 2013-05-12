#include <iostream>
#include "depthimageconverter.h"
#include "g2o/stuff/timeutil.h"

using namespace std;

DepthImageConverter::DepthImageConverter(  PointProjector* projector_,
					   HomogeneousPoint3fStatsGenerator* statsGenerator_,
					   PointOmegaGenerator* pointOmegaGenerator_,
					   NormalOmegaGenerator* normalOmegaGenerator_ ){
  _projector = projector_;
  _statsGenerator=statsGenerator_;
  _pointOmegaGenerator=pointOmegaGenerator_;
  _normalOmegaGenerator = normalOmegaGenerator_;
  
}

void DepthImageConverter::compute(HomogeneousPoint3fScene& scene, 
				  const Eigen::MatrixXf& depthImage, 
				  const Eigen::Isometry3f& sensorOffset){
  scene.clear();
  double tStart = g2o::get_time();
  // resizing the temporaries
  if (depthImage.rows()!=_indexImage.rows() ||
      depthImage.rows()!=_indexImage.cols()){
    _indexImage.resize(depthImage.rows(), depthImage.cols());
    _integralImage.resize(depthImage.rows(), depthImage.cols());
    _intervalImage.resize(depthImage.rows(), depthImage.cols());
  }
  // unprojecting
  _projector->setTransform(Eigen::Isometry3f::Identity());
  _projector->unProject(scene.points(), _indexImage, depthImage);

  scene.normals().resize(scene.points().size());
  scene.pointOmegas().resize(scene.points().size());
  scene.normalOmegas().resize(scene.points().size());
  scene.stats().resize(scene.points().size());
  std::fill(scene.stats().begin(), scene.stats().end(), HomogeneousPoint3fStats());

  // computing the integral image and the intervals
  _integralImage.compute(_indexImage,scene.points());
  _projector->projectIntervals(_intervalImage,depthImage, _normalWorldRadius);
    
  _statsGenerator->compute(scene.normals(),
			   scene.stats(),
			   scene.points(),
			   _integralImage,
			  _intervalImage,
			  _indexImage,
			  _curvatureThreshold);
  _pointOmegaGenerator->compute(scene.pointOmegas(), scene.stats(), scene.normals());
  _normalOmegaGenerator->compute(scene.normalOmegas(), scene.stats(), scene.normals());
  // scene is labeled, now we need to transform all the elements by considering the position
  // of the sensor
  //scene.transformInPlace(sensorOffset);
  double tEnd = g2o::get_time();
  //double tEnd = g2o::get_time();
  cerr << "time: " << (tEnd-tStart)*1000.0f << endl;
 
}


