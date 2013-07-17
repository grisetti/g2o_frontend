#include "bimagesensor.h"
#include "opencv2/highgui/highgui.hpp"
#include "g2o_frontend/boss/object_data.h"
#include <stdexcept>

#define BUF_BLOCK (4096*4)

namespace boss {
  

  ImageBLOB::ImageBLOB(){}

  const std::string& ImageBLOB::extension() { return _extension; }

  void ImageBLOB::resize(int width, int height, Format format_) {
    _format = format_;
    switch  (_format) {
    case mono8:  _image = cv::Mat(width, height, CV_8UC1); 
      _extension = "PGM";
       break;
    case mono16: _image = cv::Mat(width, height, CV_16UC1);
      _extension = "PGM";
       break;
    case rgb8:   _image = cv::Mat(width, height, CV_8UC3); 
      _extension = "PBM";
      break;
    }
  }


  bool ImageBLOB::read(std::istream& is) {
    std::vector<uchar> buffer;
    int count=0;
    while (is.good()){
      buffer.resize(buffer.size()+BUF_BLOCK);
      is.read((char*)&(buffer[count]),BUF_BLOCK);
      count+=is.gcount();
    }
    buffer.resize(count);
    _image = cv::imdecode(buffer, -1);
    return true;
  }

  void ImageBLOB::write(std::ostream& os) {
    std::vector<uchar> buffer;
    std::string _extension_=std::string(".")+_extension;
    bool result = cv::imencode(_extension_.c_str(), _image, buffer);
    os.write((char*)(&buffer[0]),buffer.size());
    if (! result)
      throw std::runtime_error("cv imwrite error");
  }

  ImageSensor::ImageSensor(int id, IdContext* context) :BaseSensor(id,context){}  

  Image::Image(ImageSensor* sensor_, int id, IdContext* context):
    SensorData<ImageSensor>(id, context) {
    _sensor = sensor_;
  }

  Image::~Image(){
  }

  void Image::serialize(ObjectData& data, IdContext& context) {
    SensorData<ImageSensor>::serialize(data,context);
    // alternative 1, creating an own field
    ObjectData * blobData=new ObjectData();
    data.setField("imageBlob", blobData);
    _imageBlob.serialize(*blobData,context);
    // alterative 2 embedding
    //_imageBlob.serialize(data,context);
  }
  void Image::deserialize(ObjectData& data, IdContext& context){
    SensorData<ImageSensor>::deserialize(data,context);
    ObjectData * blobData = static_cast<ObjectData *>(data.getField("imageBlob"));
    _imageBlob.deserialize(*blobData,context);
    //
    //_imageBlob.deserialize(data,context);
  }

  BOSS_REGISTER_BLOB(ImageBLOB);
  BOSS_REGISTER_CLASS(Image);
  BOSS_REGISTER_CLASS(ImageSensor);

}
