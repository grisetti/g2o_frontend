#pragma once

#include "g2o_frontend/boss_map/eigen_boss_plugin.h" 
#include "g2o_frontend/boss/blob.h"

#include "g2o_frontend/pwn_core/cloud.h"

namespace pwn_boss {
  
  class Cloud : public pwn::Cloud, public boss::BLOB {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    
    Cloud();
    virtual ~Cloud() {}

    virtual const std::string &extension();
    virtual bool read(std::istream &is);
    virtual void write(std::ostream &os);
  };

  typedef boss::BLOBReference<Cloud> CloudBLOBReference;

}
