#ifndef OMNICAMDATA_H
#define OMNICAMDATA_H

#include "sensor_data.h"
#include "sensor_omnicam.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "opencv2/highgui/highgui.hpp"

class OmnicamData : public ParameterizedSensorData{
public:
  OmnicamData(cv::Mat* image_ = 0, Sensor* _sensor=0);
  virtual ~OmnicamData();
	
  //! read the data from a stream
  virtual bool read(std::istream& is);
  //! write the data to a stream
  virtual bool write(std::ostream& os) const;
  virtual void writeOut() const ;

  void update();
  void release();
	
  const std::string& baseFilename() const { return _baseFilename; };
  void  setBaseFilename(const std::string baseFilename_) { _baseFilename = baseFilename_; };
  void setImage(cv::Mat* image_);
  const cv::Mat * getImage() const {return _image;};

protected:
  std::string _baseFilename;		// name of the image file associated with this data

private:
  mutable bool _imageModified;
  cv::Mat* _image;
};

#ifdef G2O_HAVE_OPENGL

class OmnicamDataDrawAction : public g2o::DrawAction{
public:
  OmnicamDataDrawAction() : DrawAction(typeid(OmnicamData).name()) {};
  virtual HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element,
					      g2o::HyperGraphElementAction::Parameters* params_ );
protected:
  virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
  g2o::FloatProperty* _pointSize;
};

#endif	// endif G2O_HAVE_OPENGL

#endif	// endif OMNICAMDATA_H
