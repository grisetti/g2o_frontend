#ifndef G2O_FRONTEND_DEPTH_IMAGE_DATA_H
#define G2O_FRONTEND_DEPTH_IMAGE_DATA_H

#include <iosfwd>
#include <string>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../pwn/depthimage.h"
#include "../pwn/pixelmapper.h"

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/hyper_graph_action.h"

class DepthImageData: public g2o::OptimizableGraph::Data{
 public:
  DepthImageData();
  virtual bool read(std::istream& is);
  //! write the data to a stream
  virtual bool write(std::ostream& os) const;
  void update();
  void release();
  std::string _filename;
  int paramIndex;
  long int ts_usec;
  long int ts_sec;
  DepthImage *depthImage;
};


#ifdef G2O_HAVE_OPENGL

class DepthImageDataDrawAction: public g2o::DrawAction{
 public:
  DepthImageDataDrawAction();
  virtual HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element, 
					      g2o::HyperGraphElementAction::Parameters* params_);
 protected:
  virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
  g2o::IntProperty* _beamsDownsampling;
  g2o::FloatProperty* _pointSize;  
};

#endif

#endif
