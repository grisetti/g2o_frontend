#include "pwn_imageview.h"

ColorMap::ColorMap() {
  compute (0, 255, 0xff);
}

void ColorMap::compute(int cmin, int cmax, unsigned char alpha){
  unsigned int* color = _colorMap;
  float scale = 255.0f/(float)(cmax-cmin);
  for (int i=0; i<0xffff; i++){
    unsigned char c;
    if (i<cmin) {
      c = 0xff;
    } else if (i>cmax) {
      c = 0;
      } else {
      c=(unsigned char) (255.0f-scale*(i-cmin));
    }
    *color++ = ((unsigned int) alpha << 24) | c << 16 | c << 8 | c;
  }
}


void toQImage(QImage& img, const MatrixXus& m, const ColorMap& colorMap) {
  if (img.size().width()!=m.cols() || img.size().height()!=m.rows() || img.format() != QImage::Format_ARGB32)
    img = QImage(m.cols(), m.rows(), QImage::Format_ARGB32);
  unsigned int * pix = (unsigned int*) img.bits();
  for (int i=0; i<m.rows(); i++){
    for (int j=0; j<m.cols(); j++){
      *pix++ = colorMap.color(m(i,j));
    }
  }
}
