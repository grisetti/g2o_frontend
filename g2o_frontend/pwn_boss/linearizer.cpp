#include "linearizer.h"
#include "aligner.h"

namespace pwn_boss {

  Linearizer::Linearizer(int id, boss::IdContext *context) : 
    pwn::Linearizer(), 
    boss::Identifiable(id, context) {}

  void Linearizer::serialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::serialize(data, context);
    Aligner *aligner = dynamic_cast<Aligner*>(_aligner);
    if(!aligner) {
      throw std::runtime_error("Impossible to convert pwn::Aligner to pwn_boss::Aligner");
    }
    data.setPointer("aligner", aligner);
    data.setBool("robustKernel", _robustKernel);
    data.setFloat("inlierMaxChi2", _inlierMaxChi2);
  }

  void Linearizer::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    Identifiable::deserialize(data, context);
    setRobustKernel(data.getBool("robustKernel"));
    setInlierMaxChi2(data.getFloat("inlierMaxChi2"));
    data.getReference("aligner").bind(_aligner);
  }

  void Linearizer::deserializeComplete() {}

  BOSS_REGISTER_CLASS(Linearizer);

}
