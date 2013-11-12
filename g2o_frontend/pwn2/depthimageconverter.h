#ifndef _PWN_DEPTHIMAGECONVERTER_H_
#define _PWN_DEPTHIMAGECONVERTER_H_
#include "g2o_frontend/boss_logger/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/object_data.h"
#include "g2o_frontend/boss/identifiable.h"

#include "frame.h"
#include "pointprojector.h"
#include "statscalculator.h"
#include "informationmatrixcalculator.h"

namespace pwn {

  class DepthImageConverter: public boss::Identifiable {
  public:
    DepthImageConverter(PointProjector* _projector = 0,
	StatsCalculator* _statsCalculator = 0,
	PointInformationMatrixCalculator* _pointInformationMatrixCalculator = 0,
	NormalInformationMatrixCalculator* _normalInformationMatrixCalculator = 0, 
	int id=-1, boss::IdContext* context=0);

    void compute(Frame& frame,
		 const DepthImage &depthImage, 
		 const Eigen::Isometry3f &sensorOffset = Eigen::Isometry3f::Identity(),
		 const bool blackBorders=false);

    void fastCompute(Frame& frame,
		     const DepthImage &depthImage, 
		     const Eigen::Isometry3f &sensorOffset = Eigen::Isometry3f::Identity(),
		     const int imageRadius = 3);

    // todo: add accessor methods, separate public and private
    //protected:

    PointProjector* _projector;
    StatsCalculator* _statsCalculator;
    PointInformationMatrixCalculator* _pointInformationMatrixCalculator;
    NormalInformationMatrixCalculator* _normalInformationMatrixCalculator;
  
    // this is the workspace of the object
    PointIntegralImage _integralImage;
    IntImage _indexImage;
    IntImage _intervalImage;
    IntImage _originalIndexImage;
    Eigen::MatrixXf _zBuffer;
  
    virtual void serialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserialize(boss::ObjectData& data, boss::IdContext& context);
    virtual void deserializeComplete();
  };

}

#endif
