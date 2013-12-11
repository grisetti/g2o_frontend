#include "frame.h"

namespace pwn_boss {
  using namespace boss;

  Frame::Frame() : 
    pwn::Frame() {}

  static const std::string _pwnString = "pwn";
  const std::string &Frame::extension() { 
    return _pwnString;
  }

  bool Frame::read(std::istream& is){
    Eigen::Isometry3f t;
    return this->load(t, is);
  }

  void Frame::write(std::ostream& os) {
    save(os, Eigen::Isometry3f::Identity(), 1, true);
  }

  
  BOSS_REGISTER_BLOB(Frame);

}
