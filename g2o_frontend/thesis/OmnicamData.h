#ifndef OMNICAMDATA_H
#define OMNICAMDATA_H

#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam2d/types_slam2d.h"

class OmnicamData : public g2o::HyperGraph::StampedData{
	OmnicamData(double timestamp_):StampedData(timestamp_){};
	virtual ~OmnicamData();
	//! read the data from a stream
	virtual bool read(std::istream& is);
	//! write the data to a stream
	virtual bool write(std::ostream& os) const;
	const std::string& baseFilename() const { return _baseFilename; };
	void  setBaseFilename(const std::string baseFilename_) { _baseFilename = baseFilename_; };
	cv::Mat* _image;
	
	void setPose(double x, double y, double theta);	// set the odometry
	
protected:
	std::string _baseFilename;
	g2o::ParameterCamera* _cameraParams;
private:
	double _pose[3];
	std::vector<double> observations;
	std::vector<g2o::VertexPointXY> observed_landmarks;
	int _paramIndex;
}

//	#ifdef G2O_HAVE_OPENGL
//
//	class OmnicamDataDrawAction : public g2o::DrawAction{
//	public:
//	  OmnicamDataDrawAction() : DrawAction(typeid(OmnicamData).name()) {};
//	  virtual HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element,
//	  	g2o::HyperGraphElementAction::Parameters* params_ );
//	protected:
//	  virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
//	  g2o::FloatProperty* _pointSize;
//	};
//
//	#endif	// endif G2O_HAVE_OPENGL

#endif	// endif OMNICAMDATA_H
