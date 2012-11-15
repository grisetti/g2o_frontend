/*
 * RGBDData.h
 *
 *  Created on: Nov 14, 2012
 *      Author: jacopo
 */

#ifndef RGBDDATA_H
#define RGBDDATA_H

#include <iosfwd>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "SensorRGBDCamera.h"

class RGBDData: public g2o::HyperGraph::Data {
public:  
  RGBDData();
  virtual ~RGBDData();
  //! read the data from a stream
  virtual bool read(std::istream& is);
  //! write the data to a stream
  virtual bool write(std::ostream& os) const;
  void update();
  void release();
  const std::string& baseFilename() const { return _baseFilename; };
  void  setBaseFilename(const std::string baseFilename_) { _baseFilename = baseFilename_; };
  SensorRGBDCamera* getSensor() const { return _rgbdCameraSensor; }
  cv::Mat* _intensityImage;
  cv::Mat* _depthImage;
protected:
  std::string _baseFilename;
  SensorRGBDCamera* _rgbdCameraSensor;
  long int _ts_usec;
  long int _ts_sec;
private:
  int _paramIndex;
};

#ifdef G2O_HAVE_OPENGL

class RGBDDataDrawAction : public g2o::DrawAction{
public:
  RGBDDataDrawAction() : DrawAction(typeid(RGBDData).name()) {};
  virtual HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element, 
					      																g2o::HyperGraphElementAction::Parameters* params_ );
protected:
  virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
  g2o::IntProperty* _beamsDownsampling;
  g2o::FloatProperty* _pointSize;
};

#endif

#endif
