#include "correspondencefinder.h"

namespace pwn_boss {

  CorrespondenceFinder::CorrespondenceFinder(int id, boss::IdContext *context) : 
    pwn::CorrespondenceFinder(), 
    boss::Identifiable(id, context) {}

  void CorrespondenceFinder::serialize(boss::ObjectData &data, boss::IdContext &context) {
    boss::Identifiable::serialize(data, context);
    data.setFloat("inlierDistanceThreshold", inlierDistanceThreshold());
    data.setFloat("flatCurvatureThreshold", flatCurvatureThreshold());
    data.setFloat("inlierCurvatureRatioThreshold", inlierCurvatureRatioThreshold());
    data.setFloat("inlierNormalAngularThreshold", inlierNormalAngularThreshold());
    data.setInt("rows", imageRows());
    data.setInt("cols", imageCols());
  }

  void CorrespondenceFinder::deserialize(boss::ObjectData &data, boss::IdContext &context) {
    boss::Identifiable::deserialize(data, context);
    setInlierDistanceThreshold(data.getFloat("inlierDistanceThreshold"));
    setFlatCurvatureThreshold(data.getFloat("flatCurvatureThreshold"));
    setInlierCurvatureRatioThreshold(data.getFloat("inlierCurvatureRatioThreshold"));
    setInlierNormalAngularThreshold(data.getFloat("inlierNormalAngularThreshold"));
    int r = data.getInt("rows");
    int c = data.getInt("cols");
    setImageSize(r, c);
  }

  BOSS_REGISTER_CLASS(CorrespondenceFinder);

}
