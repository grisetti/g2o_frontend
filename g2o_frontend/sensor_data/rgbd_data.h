/*
 * rgbd_data.h
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
#include "g2o/types/slam3d/types_slam3d.h"
#include "sensor_data.h"
#include "sensor_rgbd_camera.h"

class RGBDData: public ParameterizedSensorData {
public:  
  RGBDData(Sensor* sensor_ = 0 , cv::Mat* intensityImage_ = 0, cv::Mat* depthImage_ = 0);
  virtual ~RGBDData();
  //! read the data from a stream
  virtual bool read(std::istream& is);
  //! write the data to a stream
  virtual bool write(std::ostream& os) const;
  //! write the images (if changed)
  virtual void writeOut() const;

  void update();
  void release();
  inline const std::string& baseFilename() const { return _baseFilename; };
  inline void  setBaseFilename(const std::string baseFilename_) { _baseFilename = baseFilename_; };
  inline const cv::Mat* intensityImage() const {return _intensityImage;}
  inline const cv::Mat* depthImage() const {return _depthImage;}
  void setIntensityImage(cv::Mat* intensityImage_) {
    if (_intensityImage) 
      delete _intensityImage;
    _intensityImage = intensityImage_;
    _intensityImageModified =  true;
  }

  void setDepthImage(cv::Mat* depthImage_) {
    if (_depthImage) 
      delete _depthImage;
    _depthImage = depthImage_;
    _depthImageModified =  true;
  }
protected:
  std::string _baseFilename;
  cv::Mat* _intensityImage;
  cv::Mat* _depthImage;
private:
  mutable bool _intensityImageModified;
  mutable bool _depthImageModified;
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
