#include <cstdio>
#include "depthimage.h"

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
bool _readPgm(MatrixXus &image, FILE *pgmFile) {
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
  image = MatrixXus(height, width);
  int pixel;
  if (max_value>0xFF){
    for (i=0; i<height; ++i){
      for (j=0; j<width; ++j){
	hi = fgetc(pgmFile);
	lo = fgetc(pgmFile);
	pixel = (hi << 8) + lo;
	image(i, j) = pixel;
      }
    }
  }
  else{
    for (i=0; i<height; ++i){
      for (j=0; j<width; ++j){
	pixel = fgetc(pgmFile);
	image(i, j) = pixel;
      }
    }
  }
  
  return true;
}

// Write an image to a .pgm file.
bool _writePgm(const MatrixXus& img, FILE *pgmFile) {
  int i, j;
  int hi, lo;
  unsigned int max_value = 0xFFFF;
  
  if (pgmFile==NULL)
    return false;

  // Write header for a .pgm file.
  fprintf(pgmFile, "P5 ");
  fprintf(pgmFile, "%d %d ", (int)img.cols(), (int)img.rows());
  fprintf(pgmFile, "%d ", max_value);

  // Write image data.
  for (i=0; i<img.rows(); i++){
    for (j=0; j<img.cols(); j++){
      hi = HI(img(i, j));
      lo = LO(img(i, j));
      fputc(hi, pgmFile);
      fputc(lo, pgmFile);
    }
  }
  
  return true;
}


DepthImage::DepthImage(int r, int c): Eigen::MatrixXf(r,c){
  fill(std::numeric_limits<float>::max());
}

DepthImage::DepthImage(const MatrixXus& m): Eigen::MatrixXf(m.rows(),m.cols()){
    fromUnsignedShort(m);
}

void DepthImage::toUnsignedShort(MatrixXus& m, float dmax) const {
  m.resize(rows(), cols());
  unsigned short* us=m.data();
  const float* f=data();
  int s = m.rows()*m.cols();
  for (int i =0; i<s; i++, f++, us++)
    *us = (*f<dmax) ? (int)(1000.0f*(*f)) : 0;
}

void DepthImage::fromUnsignedShort(const MatrixXus& m){
  resize(m.rows(), m.cols());
  const unsigned short* us=m.data();
  float* f=data();
  int s = m.rows()*m.cols();
  for (int i =0; i<s; i++, f++, us++)
    *f = (*us) ? 0.001f*(*us) : std::numeric_limits<float>::max();
}

 bool DepthImage::load(const char* filename){
   MatrixXus usm;
   FILE* f=fopen(filename, "rb");
   bool result = _readPgm(usm, f);
   if (! result)
     return false;
   fromUnsignedShort(usm);
   return true;
 }

 bool DepthImage::save(const char* filename) const{
   MatrixXus usm;
   toUnsignedShort(usm, 15.0f);
   FILE* f=fopen(filename, "wb");
   return _writePgm(usm, f);
}

