#include <iostream>
#include "depthimageconverter.h"
#include "pinholepointprojector.h"
#include "g2o/stuff/timeutil.h"

#include <fstream>

using namespace std;

namespace pwn {
  using namespace boss;
  DepthImageConverter::DepthImageConverter(PointProjector *projector_,
					   StatsCalculator *statsCalculator_,
					   PointInformationMatrixCalculator *pointInformationMatrixCalculator_,
					   NormalInformationMatrixCalculator *normalInformationMatrixCalculator_,
	int id, boss::IdContext* context): Identifiable(id,context) {
    _projector = projector_;
    _statsCalculator = statsCalculator_;
    _pointInformationMatrixCalculator = pointInformationMatrixCalculator_;
    _normalInformationMatrixCalculator = normalInformationMatrixCalculator_;
  }

  void DepthImageConverter::compute(Frame &frame,
				    const DepthImage &depthImage, 
				    const Eigen::Isometry3f &sensorOffset,
				    const bool blackBorders) {
    const float _normalWorldRadius = _statsCalculator->worldRadius();
    frame.clear();
    // resizing the temporaries
    if (depthImage.rows()!=_indexImage.rows() ||
	depthImage.cols()!=_indexImage.cols()){
      _indexImage.resize(depthImage.rows(), depthImage.cols());
      _integralImage.resize(depthImage.rows(), depthImage.cols());
      _intervalImage.resize(depthImage.rows(), depthImage.cols());
    }
    // unprojecting
    _projector->setTransform(Eigen::Isometry3f::Identity());
    _projector->unProject(frame.points(), frame.gaussians(), _indexImage, depthImage);

    frame.normals().resize(frame.points().size());
    frame.pointInformationMatrix().resize(frame.points().size());
    frame.normalInformationMatrix().resize(frame.points().size());
    frame.stats().resize(frame.points().size());
    std::fill(frame.stats().begin(), frame.stats().end(), Stats());

    // computing the integral image and the intervals
    _integralImage.compute(_indexImage,frame.points());
    _projector->projectIntervals(_intervalImage,depthImage, _normalWorldRadius, blackBorders);

    _statsCalculator->compute(frame.normals(),
			      frame.stats(),
			      frame.points(),
			      _integralImage,
			      _intervalImage,
			      _indexImage);
    _pointInformationMatrixCalculator->compute(frame.pointInformationMatrix(), frame.stats(), frame.normals());
    _normalInformationMatrixCalculator->compute(frame.normalInformationMatrix(), frame.stats(), frame.normals());

    frame.transformInPlace(sensorOffset);
  }

  void DepthImageConverter::serialize(boss::ObjectData& data, boss::IdContext& context) {
    Identifiable::serialize(data,context);
    data.setPointer("pointProjector", _projector);
    data.setPointer("statsCalculator", _statsCalculator);
    data.setPointer("pointInfoCalculator", _pointInformationMatrixCalculator);
    data.setPointer("normalInfoCalculator", _normalInformationMatrixCalculator);
  }
  
  void DepthImageConverter::deserialize(boss::ObjectData& data, boss::IdContext& context) {
    Identifiable::deserialize(data,context);
    data.bindPointer("pointProjector", _tempProjector);
    data.bindPointer("statsCalculator", _tempStatsCalculator);
    data.bindPointer("pointInfoCalculator", _tempPointInfoCalculator);
    data.bindPointer("normalInfoCalculator", _tempNormalInfoCalculator);
  }
  
  void DepthImageConverter::deserializeComplete(){
    _projector=dynamic_cast<PointProjector*>(_tempProjector);
    _statsCalculator=dynamic_cast<StatsCalculator*>(_tempStatsCalculator);
    _pointInformationMatrixCalculator=dynamic_cast< PointInformationMatrixCalculator* >(_tempPointInfoCalculator);
    _tempNormalInfoCalculator=dynamic_cast< NormalInformationMatrixCalculator*> (_tempNormalInfoCalculator);
  }

  BOSS_REGISTER_CLASS(DepthImageConverter);

}
