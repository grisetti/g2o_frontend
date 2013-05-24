#ifndef _G2O_FRONTEND_LINE2D_VALIDATOR_H_
#define _G2O_FRONTEND_LINE2D_VALIDATOR_H_
#include "ransac.h"
#include "g2o/types/slam2d_addons/vertex_line2d.h"

namespace g2o_frontend{
  
  class Line2DCorrespondenceValidator: public CorrespondenceValidator{
  public:
    Line2DCorrespondenceValidator() :CorrespondenceValidator(2){
      _intraFrameDistanceDifference=.1;
      _intraFrameMinimalDistance=1.;
    }
    void setIntraFrameDistanceDifference(double idd){ _intraFrameDistanceDifference = idd;}
    void setIntraFrameMinimalDistance(double md){ _intraFrameMinimalDistance = md;}
    double intraFrameDistanceDifference() const {return _intraFrameDistanceDifference;}
    double intraFrameMinimalDistance() const {return _intraFrameMinimalDistance;}

    virtual bool operator()(const CorrespondenceVector& correspondences, const IndexVector& indices, int k);
  protected:
    double _intraFrameDistanceDifference;
    double _intraFrameMinimalDistance;

  };

}

#endif
