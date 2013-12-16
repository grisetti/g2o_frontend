#include "cloud.h"

namespace pwn_boss {
  using namespace boss;

  Cloud::Cloud() : 
    pwn::Cloud() {}

  static const std::string _pwnString = "pwn";
  const std::string &Cloud::extension() { 
    return _pwnString;
  }

  bool Cloud::read(std::istream &is){
    Eigen::Isometry3f t;
    return this->load(t, is);
  }

  void Cloud::write(std::ostream &os) {
    save(os, Eigen::Isometry3f::Identity(), 1, true);
  }

  
  BOSS_REGISTER_BLOB(Cloud);

}
