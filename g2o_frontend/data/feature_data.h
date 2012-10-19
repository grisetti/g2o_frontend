// g2o - General Graph Optimization
// Copyright (C) 2011 R. Kuemmerle, G. Grisetti, W. Burgard
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#ifndef G2O_FRONTEND_FEATURE_DATA_H
#define G2O_FRONTEND_FEATURE_DATA_H

#include <iosfwd>
#include <string>

#include "g2o/core/optimizable_graph.h"
#include "g2o/core/hyper_graph_action.h"
#include "g2o/types/slam2d/types_slam2d.h"


struct BaseFeatureData: public g2o::OptimizableGraph::Data{
  virtual bool read(std::istream& is);
  //! write the data to a stream 
  virtual bool write(std::ostream& os) const;
  virtual int positionMeasurementDimension() const = 0;
  virtual void positionMeasurement(double*) const = 0;
  virtual void setPositionMeasurement(const double*) = 0;
  virtual void positionMeasurementInformation(double*) const = 0;
  virtual void setPositionMeasurementInformation(const double*) = 0;
  virtual int appearanceMeasurementDimension() const = 0;
  virtual void appearanceMeasurement(double*) const = 0;
  virtual void setAppearanceMeasurement(const double*) = 0;
  virtual g2o::OptimizableGraph::Vertex* constructLandmarkVertex() const = 0;
  virtual g2o::OptimizableGraph::Edge* constructEdge() const = 0;
};

struct FeaturePointXYData: public BaseFeatureData{
  virtual int positionMeasurementDimension() const;
  virtual void positionMeasurement(double*) const;
  virtual void setPositionMeasurement(const double*);
  virtual void positionMeasurementInformation(double*) const;
  virtual void setPositionMeasurementInformation(const double*);
  virtual int appearanceMeasurementDimension() const;
  virtual void appearanceMeasurement(double*) const;
  virtual void setAppearanceMeasurement(const double*);
  virtual g2o::OptimizableGraph::Vertex* constructLandmarkVertex() const;
  virtual g2o::OptimizableGraph::Edge* constructEdge() const;
  inline const Eigen::Vector2d& positionMeasurement() const { return _positionMeasurement ;}
  inline void setPositionMeasurement(const Eigen::Vector2d& v) { _positionMeasurement=v ;}
  inline const Eigen::Matrix2d& positionInformation() const { return _positionInformation;}
  inline void setPositionInformation(const Eigen::Matrix2d& v) { _positionInformation=v ;}
 protected:
  Eigen::Vector2d _positionMeasurement;
  Eigen::Matrix2d _positionInformation;
};


#ifdef G2O_HAVE_OPENGL

class FeaturePointXYDataDrawAction: public g2o::DrawAction{
 public:
  FeaturePointXYDataDrawAction();
  virtual HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element,
					      g2o::HyperGraphElementAction::Parameters* params_ );
 protected:
  virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_);
  g2o::FloatProperty* _pointSize;
  g2o::FloatProperty* _lineWidth;
};

#endif

#endif
