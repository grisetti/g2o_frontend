#include "pwn_io.h"
#include <iostream>
#include "g2o_frontend/boss/deserializer.h"

namespace pwn_boss {
  
  using namespace std;

  std::vector<boss::Serializable*> readConfig(pwn_boss::Aligner*& aligner, pwn_boss::DepthImageConverter*& converter, const std::string& configFile){
    aligner = 0;
    converter = 0;
    boss::Deserializer des;
    des.setFilePath(configFile);
    boss::Serializable* s;
    std::vector<boss::Serializable*> instances;
    cerr << "Reading" << endl;
    while ((s=des.readObject())){
      instances.push_back(s);
      pwn_boss::Aligner* al=dynamic_cast<pwn_boss::Aligner*>(s);
      if (al) {
	cerr << "got aligner" << endl;
	aligner = al;
      }
      pwn_boss::DepthImageConverter* conv=dynamic_cast<pwn_boss::DepthImageConverter*>(s);
      if  (conv) {      
	cerr << "got converter" << endl;
	converter = conv;
      }
    }
    if (aligner) {
      cerr << "alpp: " << aligner->projector() << endl;
      cerr << "allz: " << aligner->linearizer() << endl;
      if (aligner->linearizer())
	cerr << "lzal: " << aligner->linearizer()->aligner() << endl;
    
    }

    return instances;
  }

}
