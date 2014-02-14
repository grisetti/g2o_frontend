#include "drawable.h"


using namespace Eigen;



Drawable::Drawable()
{
    _transformation = Isometry3f::Identity();
    _viewer = 0;
}


Drawable::Drawable(Isometry3f transformation_)
{
    _transformation = transformation_;
    _viewer = 0;
}
