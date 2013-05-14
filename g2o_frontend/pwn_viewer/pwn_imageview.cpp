#include "pwn_imageview.h"

DepthImageView::DepthImageView() {
  computeColorMap (0, 255, 0xff);
}

void DepthImageView::computeColorMap(int cmin, int cmax, unsigned char alpha){
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

void DepthImageView::convertToQImage(QImage& img, const MatrixXus& m) const {
  if (img.size().height()!=m.cols() || img.size().width()!=m.rows() || img.format() != QImage::Format_ARGB32)
    img = QImage(m.rows(), m.cols(), QImage::Format_ARGB32);
  for (int i=0; i<m.rows(); i++){
    for (int j=0; j<m.cols(); j++){
      img.setPixel(i, j, color(m(i, j)));
    }
  }
}

void DepthImageView::convertToQImage(QImage& img, const Eigen::MatrixXf& m) const {
  if (img.size().height()!=m.cols() || img.size().width()!=m.rows() || img.format() != QImage::Format_ARGB32)
    img = QImage(m.rows(), m.cols(), QImage::Format_ARGB32);
  for (int i=0; i<m.rows(); i++){
    for (int j=0; j<m.cols(); j++){
      img.setPixel(i, j, color(1000.0f*m(i, j)));
    }
  }
}
