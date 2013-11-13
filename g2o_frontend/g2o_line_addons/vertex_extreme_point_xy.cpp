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

#include "vertex_extreme_point_xy.h"
#include "g2o/stuff/opengl_wrapper.h"
#include "g2o/stuff/opengl_primitives.h"
#include <iostream>

namespace g2o {

  VertexExtremePointXY::VertexExtremePointXY() :
    VertexPointXY()
  {
    _estimate.setZero();
    _isExtremePoint = 0.5;
  }


  bool VertexExtremePointXY::read(std::istream& is)
  {
    is >> _estimate[0] >> _estimate[1] >> _isExtremePoint;
    return true;
  }

  bool VertexExtremePointXY::write(std::ostream& os) const
  {
      os << estimate()(0) << " " << estimate()(1) << " " << isExtremePoint();
    return os.good();
  }

  VertexExtremePointXYWriteGnuplotAction::VertexExtremePointXYWriteGnuplotAction(): WriteGnuplotAction(typeid(VertexExtremePointXY).name()){}

  HyperGraphElementAction* VertexExtremePointXYWriteGnuplotAction::operator()(HyperGraph::HyperGraphElement* element, HyperGraphElementAction::Parameters* params_){
    if (typeid(*element).name()!=_typeName)
      return 0;

    WriteGnuplotAction::Parameters* params=static_cast<WriteGnuplotAction::Parameters*>(params_);
    if (!params->os){
      std::cerr << __PRETTY_FUNCTION__ << ": warning, on valid os specified" << std::endl;
      return 0;
    }

    VertexExtremePointXY* v =  static_cast<VertexExtremePointXY*>(element);
    *(params->os) << v->estimate().x() << " " << v->estimate().y() << std::endl;
    return this;
  }

  VertexExtremePointXYDrawAction::VertexExtremePointXYDrawAction(): DrawAction(typeid(VertexExtremePointXY).name()){}

  bool VertexExtremePointXYDrawAction::refreshPropertyPtrs(HyperGraphElementAction::Parameters* params_){
    if (! DrawAction::refreshPropertyPtrs(params_))
      return false;
    if (_previousParams){
      _pointSize = _previousParams->makeProperty<FloatProperty>(_typeName + "::POINT_SIZE", 1.);
    } else {
      _pointSize = 0;
    }
    return true;
  }

  HyperGraphElementAction* VertexExtremePointXYDrawAction::operator()(HyperGraph::HyperGraphElement* element,
                     HyperGraphElementAction::Parameters* params ){

    if (typeid(*element).name()!=_typeName)
      return 0;
    initializeDrawActionsCache();
    refreshPropertyPtrs(params);
    if (! _previousParams)
      return this;

    if (_show && !_show->value())
      return this;
    VertexExtremePointXY* that = static_cast<VertexExtremePointXY*>(element);


    glPushMatrix();
    glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
    glDisable(GL_LIGHTING);
    glColor3f(LANDMARK_VERTEX_COLOR);
    float ps = _pointSize ? _pointSize->value() :  1.0f;
    glTranslatef((float)that->estimate()(0),(float)that->estimate()(1),0.0f);
    opengl::drawPoint(ps);
    glPopAttrib();
    drawCache(that->cacheContainer(), params);
    drawUserData(that->userData(), params);
    glPopMatrix();
    return this;
  }


} // end namespace
