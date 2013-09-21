#include <iostream>
#include "cv.h"
#include "highgui.h"
#include <vector>

using namespace std;

float compareImages(cv::Mat& m1, cv::Mat& m2){
  if (m1.rows!=m2.rows || m1.cols != m2.cols)
    return 0;
  return norm(m1-m2);
}

std::vector<cv::Mat> images;
cv::Mat comparison;
int r = 0;
int c = 0;
float cutoff = 0;

void mouseEvent(int evt, int x, int y, int flags, void* param){
    if(evt==CV_EVENT_LBUTTONDOWN){
      r = x;
      c = y;
      cv::Mat& m1 = images[r];
      cv::Mat& m2 = images[c];
      printf("%d %d %f\n",r,c, comparison.at<float>(r,c));
      cv::imshow("image1", m1);
      cv::imshow("image2", m2);
    }
}

int main(int argc, char** argv){

  for (int i=1; i<argc; i++){
    cv::Mat img = cv::imread(argv[i]);
    img.convertTo(img,CV_32FC3);
    img=img-127.0f;
    img=img*(1./255);
    cv::resize(img, img, cv::Size(img.cols, img.rows));
    images.push_back(img);
  }
  cerr << "read " << images.size() << " images" << endl;
 
  comparison = cv::Mat(images.size(), images.size(), CV_32F);
  float max = 0;
  for (int i = 0; i<comparison.rows; i++)
    for (int j = i; j<comparison.cols; j++){
      cv::Mat& m1  = images[i];
      cv::Mat& m2  = images[j];
      float f = compareImages(m1,m2);
      //cerr << "val: " << i << " " << j << " " << f << endl;
      comparison.at<float>(i,j)=f;
      if (max<f)
	max = f;
    }
  cerr << "max: " << max;
  max = 1./max;
  for (int i = 0; i<comparison.rows; i++)
    for (int j = i; j<comparison.cols; j++){
      comparison.at<float>(i,j)*=max;
    }
  cv::imwrite("comparison.pgm", comparison);
  cvNamedWindow("comparison", 0);
  cvNamedWindow("image1", 0);
  cvNamedWindow("image2", 0);
  cvNamedWindow("diff", 0);
  cvSetMouseCallback("comparison", mouseEvent, 0);
  
  int ca = 0;
  while(ca!=27){
    cv::Mat shownImage = comparison.clone();
    cv::Mat cutoffImage = comparison>cutoff;
    //
cv::Mat shownImage = cutoffImage;
    int nz = images.size()*images.size() -cv::countNonZero(cutoffImage);
    nz -= images.size()*(images.size()+1)/2;
    cv::circle(shownImage, cv::Point(r,c), 4, 0.0f);
    cv::imshow("comparison", shownImage);
    ca = cv::waitKey();
    switch (ca) {
    case 1113938: c --; break;
    case 1113940: c ++; break;
    case 1113937: r --; break;
    case 1113939: r ++; break;
    case 1114027: cutoff += 0.01; break;
    case 1114029: cutoff -= 0.01; break;
    default: std::cerr << ca << endl;
    }
    if (r>=images.size())
      r = images.size()-1;
    if (c>=images.size())
      c = images.size()-1;
    if (r<0)
      r = 0;
    if (c<0)
      c = 0;
    cv::Mat& m1 = images[r];
    cv::Mat& m2 = images[c];
    printf("%d %d %f %f %d\n",r,c, comparison.at<float>(c,r), cutoff, nz);
    cv::imshow("image1", m1);
    cv::imshow("image2", m2);
    cv::Mat diff = abs(m1-m2);
    cv::imshow("diff", diff);
  }

  return 0;
}
