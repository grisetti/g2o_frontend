#ifndef _G2O_FRONTEND_ID_VALIDATOR_H_
#define _G2O_FRONTEND_ID_VALIDATOR_H_
#include "ransac.h"
namespace g2o_frontend{

  class IdCorrespondenceValidator: public CorrespondenceValidator{
  public:
    IdCorrespondenceValidator(int mss) :CorrespondenceValidator(mss){}
    virtual bool operator()(const CorrespondenceVector& correspondences, const IndexVector& indices, int k);

  };

}

#endif
