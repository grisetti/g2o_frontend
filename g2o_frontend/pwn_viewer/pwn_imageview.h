#ifndef PWN_IMAGEVIEW_H_
#define PWN_IMAGEVIEW_H_

#include "g2o_frontend/pwn2/depthimage.h"
#include <qimage.h>
#include <qlabel.h>

using namespace pwn;

struct DepthImageView{
  DepthImageView();
  void computeColorMap(int cmin, int cmax, unsigned char alpha);
  inline unsigned int color (unsigned short idx) const {
    return _colorMap[idx];
  }
  void convertToQImage(QImage& img, const MatrixXus& m) const;
  void convertToQImage(QImage& img, const Eigen::MatrixXf& m) const;
  
protected:
  unsigned int _colorMap[0xffff];
};


#endif
