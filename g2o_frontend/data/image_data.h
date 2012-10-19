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

#ifndef G2O_FRONTEND_IMAGE_DATA_H
#define G2O_FRONTEND_IMAGE_DATA_H

#include <iosfwd>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "g2o/core/optimizable_graph.h"
#include "g2o/core/hyper_graph_action.h"


struct ImageData: public g2o::OptimizableGraph::Data{
  virtual bool read(std::istream& is);
  //! write the data to a stream
  virtual bool write(std::ostream& os) const;
  void update();
  void release();
  std::string _imageFilename;
  int paramIndex;
  long int ts_usec;
  long int ts_sec;
  cv::Mat*_image;
};


/* #ifdef G2O_HAVE_OPENGL */

/* class PointCloudDataDrawAction: public g2o::DrawAction{ */
/*  public: */
/*   PointCloudDataDrawAction(); */
/*   virtual HyperGraphElementAction* operator()(g2o::HyperGraph::HyperGraphElement* element, */
/* 					      g2o::HyperGraphElementAction::Parameters* params_ ); */
/*  protected: */
/*   virtual bool refreshPropertyPtrs(g2o::HyperGraphElementAction::Parameters* params_); */
/*   g2o::IntProperty* _beamsDownsampling; */
/*   g2o::FloatProperty* _pointSize; */
  
/* }; */

/* #endif */

#endif
