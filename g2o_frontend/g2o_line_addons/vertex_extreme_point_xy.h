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

#ifndef G2O_VERTEX_EXTREME_POINT_XY_H
#define G2O_VERTEX_EXTREME_POINT_XY_H

#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/config.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>

namespace g2o {

  class G2O_TYPES_SLAM2D_API VertexExtremePointXY : public VertexPointXY
  {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      VertexExtremePointXY();

      virtual void setToOriginImpl() {
        _estimate.setZero();
        _isExtremePoint = 0.5;
      }

      virtual bool read(std::istream& is);
      virtual bool write(std::ostream& os) const;

      // 1 if it's a fake extreme point, 0 otherwise
      double isExtremePoint() const { return _isExtremePoint;}
      void setIsExtremePoint(double isExtremePoint){ _isExtremePoint = isExtremePoint;}

  protected:
      double _isExtremePoint;
  };

  class G2O_TYPES_SLAM2D_API VertexExtremePointXYWriteGnuplotAction: public WriteGnuplotAction {
  public:
    VertexExtremePointXYWriteGnuplotAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
            HyperGraphElementAction::Parameters* params_);
  };

  class G2O_TYPES_SLAM2D_API VertexExtremePointXYDrawAction: public DrawAction{
  public:
    VertexExtremePointXYDrawAction();
    virtual HyperGraphElementAction* operator()(HyperGraph::HyperGraphElement* element,
            HyperGraphElementAction::Parameters* params_);
  protected:
    FloatProperty *_pointSize;
    virtual bool refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_);
  };

}

#endif
