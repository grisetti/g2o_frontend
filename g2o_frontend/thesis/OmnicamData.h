#ifndef OMNICAMDATA_H
#define OMNICAMDATA_H

#include "StampedData.h"
#include "g2o/core/hyper_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"
#include "opencv2/highgui/highgui.hpp"

class OmnicamData : public StampedData{
public:
	int _nextId; // NOTE: we must find a way to get the id for the next Vertex to be added
	OmnicamData();
	OmnicamData(double timestamp_);
	virtual ~OmnicamData();
	//! read the data from a stream
	virtual bool read(std::istream& is);
	//! write the data to a stream
	virtual bool write(std::ostream& os) const;
	const std::string& baseFilename() const { return _baseFilename; };
	void  setBaseFilename(const std::string baseFilename_) { _baseFilename = baseFilename_; };
	cv::Mat* _image;
	
	void setPose(double x, double y, double theta);		// sets the odometry
	void addObservation(double bearing, g2o::VertexPointXY landmark);	// adds an observation to the observation vector, and the assigned landmark to the observed_landmarks vector
protected:
	std::string _baseFilename;		// name of the image file associated with this data
	g2o::ParameterCamera* _cameraParams;	// pointer to the camera parametres
private:
	double _pose[3];
	std::vector<double> _observations;
	std::vector<g2o::VertexPointXY> _observed_landmarks;
	int _paramIndex;
};

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
