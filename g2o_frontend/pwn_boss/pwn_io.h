#pragma once
#include "aligner.h"
#include "depthimageconverter.h"

namespace pwn_boss {
  
  std::vector<boss::Serializable*> readConfig(pwn_boss::Aligner*& aligner, pwn_boss::DepthImageConverter*& converter, const std::string& configFile);

}
