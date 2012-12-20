#ifndef DM_IMAGEVIEW_H_
#define DM_IMAGEVIEW_H_
#include "../dm_optimization/dm_cloud.h"
#include <qimage.h>
#include <qlabel.h>

struct ColorMap{
  ColorMap();
  void compute(int cmin, int cmax, unsigned char alpha);
  inline unsigned int color (unsigned short idx) const {
    return _colorMap[idx];
  }
protected:
  unsigned int _colorMap[0xffff];
};

void toQImage(QImage& img, const MatrixXus& m, const ColorMap& colorMap);

#endif
