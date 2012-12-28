#include "distance_correspondence_validator.h"

namespace g2o_frontend{
  using namespace g2o;
  DistanceCorrespondenceValidatorPointXY::DistanceCorrespondenceValidatorPointXY():
    DistanceCorrespondenceValidator<VertexPointXY>(2){
  }

  DistanceCorrespondenceValidatorPointXYZ::DistanceCorrespondenceValidatorPointXYZ():
    DistanceCorrespondenceValidator<VertexPointXYZ>(3){
  }
  
}
