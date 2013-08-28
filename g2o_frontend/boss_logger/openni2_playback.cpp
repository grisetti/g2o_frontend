#include <sstream>
#include <stdexcept>

#include "g2o_frontend/boss/deserializer.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss_logger/bimagesensor.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace boss;
using namespace cv;

PinholeImageSensor* imageSensor=0;

int main(int argc, char **argv)
{
  
  namedWindow("xtion",1);
  std::vector<Message*> messages;
  Deserializer des;
  Serializer ser;
  des.setFilePath("./fava.log");
  Message* m;
  while ((m=des.readMessage())){
    messages.push_back(m);
  }

  for (size_t i=0; i<messages.size();  i++){
    cout << "I read " << messages.size() << " messages" << endl;
    Message* m=messages[i];
    PinholeImageData* img=dynamic_cast<PinholeImageData*>(m->getInstance());
    if (img) {
      char c = waitKey();
      ImageBLOB* ib=img->imageBlob().get();
      cerr << "ib: " << img->imageBlob().getFileName() << " " << ib->cvImage().rows << "x" << ib->cvImage().cols << endl;
      imshow("xtion",ib->cvImage());
      delete ib;
    }
  }

  cerr << "writing" << endl;
  ser.setFilePath("fava.log.check");
  for (size_t i=0;i<messages.size();i++) {
    ser.write(*messages[i]);
  }
  cout << "I wrote again " << messages.size() << " messages" << endl;
  return 0;
}
