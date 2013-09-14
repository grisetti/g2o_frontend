#include "depthimage.h"
#include <cstdio>

namespace pwn {

#define HI(num) (((num) & 0x0000FF00) >> 8)
#define LO(num) ((num) & 0x000000FF)

// Skip possible comments when reading a file .pgm
void _skipComments(FILE *fp)
{
  int ch;
  char line[1024];
  char* fgetsRes;
  
  // Each comment begin with a '#', skip all lines
  // that begin with a sharp.
  while ((ch = fgetc(fp)) != EOF && isspace(ch));
  if (ch=='#'){
    fgetsRes = fgets(line, sizeof(line), fp);
    if(fgetsRes)
      _skipComments(fp);
  }
  else
    fseek(fp, -1, SEEK_CUR);
}

// Read and load an image from a .pgm file.
bool _readPgm(MatrixXus &image, FILE *pgmFile, bool transposed) {
  char version[3];
  int height, width;
  int max_value;
  int i, j;
  int lo, hi;
  char *fgetsRes;
  int fscanfRes;
 
  if (pgmFile==NULL)
    return false;
  
  // Check if it's a pgm image.
  fgetsRes = fgets(version, sizeof(version), pgmFile);
  if(fgetsRes && strcmp(version, "P5"))
    return false;
  
  // Read (width, height) of the image and the max gray value for a pixel.
  // Do this while skipping possible comments.
  _skipComments(pgmFile);
  fscanfRes = fscanf(pgmFile, "%d", &width);
  if (fscanfRes != 1)
    return false;
  _skipComments(pgmFile);
  fscanfRes = fscanf(pgmFile, "%d", &height);
  if (fscanfRes != 1)
    return false;
  _skipComments(pgmFile);
  fscanfRes = fscanf(pgmFile, "%d", &max_value);
  if (fscanfRes != 1)
    return false;
  fgetc(pgmFile);
  
  // Read image data (expected 16 bit unsigned char).
  if (transposed) {
    image.resize(width,height);
  } else {
    image.resize(height, width);
  }
  int pixel;
  if (max_value>0xFF){
    for (i=0; i<height; ++i){
      for (j=0; j<width; ++j){
	hi = fgetc(pgmFile);
	lo = fgetc(pgmFile);
	pixel = (hi << 8) + lo;
	if (transposed)
	  image(j, i) = pixel;
	else
	  image(i, j) = pixel;
      }
    }
  }
  else{
    for (i=0; i<height; ++i){
      for (j=0; j<width; ++j){
	pixel = fgetc(pgmFile);
	if (transposed)
	  image(j, i) = pixel;
	else
	  image(i, j) = pixel;
      }
    }
  }
  return true;
}

// Write an image to a .pgm file.
bool _writePgm(const MatrixXus &img, FILE *pgmFile, bool transposed) {
  int i, j;
  int hi, lo;
  unsigned int max_value = 0xFFFF;
  
  if (pgmFile==NULL)
    return false;

  // Write header for a .pgm file.
  fprintf(pgmFile, "P5 ");
  if (transposed)
    fprintf(pgmFile, "%d %d ", (int)img.rows(), (int)img.cols());
  else
    fprintf(pgmFile, "%d %d ", (int)img.cols(), (int)img.rows());
  fprintf(pgmFile, "%d ", max_value);

  // Write image data.
  if (transposed){
    for (j=0; j<img.cols(); j++) {
      for (i=0; i<img.rows(); i++){
	hi = HI(img(i, j));
	lo = LO(img(i, j));
	fputc(hi, pgmFile);
	fputc(lo, pgmFile);
      }
    }
  } else {
    for (i=0; i<img.rows(); i++){
      for (j=0; j<img.cols(); j++){
	hi = HI(img(i, j));
	lo = LO(img(i, j));
	fputc(hi, pgmFile);
	fputc(lo, pgmFile);
      }
    }
  }
  
  return true;
}


DepthImage::DepthImage(int r, int c): Eigen::MatrixXf(r,c){
  fill(std::numeric_limits<float>::max());
}

  DepthImage::DepthImage(const MatrixXus &m, float scaleFactor): Eigen::MatrixXf(m.rows(),m.cols()){
    fromUnsignedShort(m, scaleFactor);
}

void DepthImage::scale(Eigen::MatrixXf& dest, const Eigen::MatrixXf& src, int step){
  int rows = src.rows()/step;
  int cols = src.cols()/step;
  dest.resize(rows,cols);
  dest.fill(0);
  for (int c = 0; c<dest.cols(); c++){
    for (int r = 0; r<dest.rows(); r++){
      float acc=0;
      int np=0;
      int sc = c*step;
      int sr = r*step;
      for (int i=0; i<step; i++){
	for (int j=0; j<step; j++){
	  if (sc + i < src.cols()&&
	      sr + j < src.rows()) {
	    acc += src(sr+j,sc+i);
	    np += src(sr+j,sc+i) > 0;
	  }
	}
      }
      if (np)
	dest(r,c) = acc/np;
    }
  }
}


  void DepthImage::toUnsignedShort(MatrixXus &m, float dmax, float scaleFactor) const {
  m.resize(rows(), cols());
  unsigned short* us=m.data();
  const float* f=data();
  int s = m.rows()*m.cols();
  for (int i =0; i<s; i++, f++, us++) {
    *us = (*f<dmax) ? (int)((*f)/scaleFactor) : 0;
  }
}

  void DepthImage::fromUnsignedShort(const MatrixXus &m, float scaleFactor){
  resize(m.rows(), m.cols());
  const unsigned short* us=m.data();
  float* f=data();
  int s = m.rows()*m.cols();
  for (int i =0; i<s; i++, f++, us++)
    *f = (*us) ? scaleFactor*(*us) : std::numeric_limits<float>::max();
}

  void DepthImage::toCvMat(cv::Mat &m, float dmax) const{
    if (m.rows != cols() && m.cols != rows() && m.type()!=CV_16UC1){
      m=cv::Mat(cols(),rows(), CV_16UC1);
    }
    //Eigen::MatrixXf t=transpose();
    const DepthImage& t=*this;
    unsigned short* us=(unsigned short*)m.data;
    const float* f=t.data();
    int s = m.rows*m.cols;
    for (int i =0; i<s; i++, f++, us++) {
      *us = (*f<dmax) ? (int)(1000.0f*(*f)) : 0;
    }
  }
  
  void DepthImage::fromCvMat(const cv::Mat &m){
    assert (m.type==CV_16UC1);
    resize(m.cols,m.rows);
    const unsigned short* us=(const unsigned short*)m.data;
    float* f=data();
    int s = m.rows*m.cols;
    for (int i =0; i<s; i++, f++, us++)
      *f = (*us) ? 0.001f*(*us) : std::numeric_limits<float>::max();
    //transposeInPlace();
  }
  
  bool DepthImage::load(const char* filename, bool transposed, float scaleFactor) {
   MatrixXus usm;
   FILE* f=fopen(filename, "rb");
   bool result = _readPgm(usm, f, transposed);
   fclose(f);
   if (! result)
     return false;
   fromUnsignedShort(usm, scaleFactor);
   return true;
 }

  bool DepthImage::save(const char* filename, bool transposed, float scaleFactor) const{
   MatrixXus usm;
   toUnsignedShort(usm, 15.0f, scaleFactor);
   FILE* f=fopen(filename, "wb");
   return _writePgm(usm, f, transposed);
}

}

