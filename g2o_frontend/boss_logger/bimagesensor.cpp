#include "bimagesensor.h"
#include "opencv2/highgui/highgui.hpp"
#include "g2o_frontend/boss/object_data.h"
#include <stdexcept>

#define BUF_BLOCK (4096*4)

namespace boss {
  
  using namespace std;

  ImageBLOB::ImageBLOB(){}

  const std::string& ImageBLOB::extension() { return _extension; }

  void ImageBLOB::resize(int width, int height, Format format_) {
    _format = format_;
    switch  (_format) {
    case mono8:  _image = cv::Mat(width, height, CV_8UC1); 
      _extension = "pgm";
       break;
    case mono16: _image = cv::Mat(width, height, CV_16UC1);
      _extension = "pgm";
       break;
    case rgb8:   _image = cv::Mat(width, height, CV_8UC3); 
      _extension = "pbm";
      break;
    }
  }

  void ImageBLOB::adjustFormat() {
    switch (cvImage().type()){
    case CV_8UC1: 
      _format = mono8;
      break;
    case CV_16UC1: 
      _format = mono16;
      break;
    case CV_8UC3:
      _format = rgb8;
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
  ImageSensor::~ImageSensor(){}

  ImageData::ImageData(ImageSensor* sensor_, int id, IdContext* context):
    SensorData<ImageSensor>(id, context) {
    _sensor = sensor_;
  }

  ImageData::~ImageData(){
  }

  void ImageData::serialize(ObjectData& data, IdContext& context) {
    SensorData<ImageSensor>::serialize(data,context);
    // alternative 1, creating an own field
    ObjectData * blobData=new ObjectData();
    data.setField("imageBlob", blobData);
    _imageBlob.serialize(*blobData,context);
  }
  void ImageData::deserialize(ObjectData& data, IdContext& context){
    //cerr << "A";
    SensorData<ImageSensor>::deserialize(data,context);
    // alternative 1, creating an own field
    //cerr << "B";
    ObjectData * blobData = static_cast<ObjectData *>(data.getField("imageBlob"));
    _imageBlob.deserialize(*blobData,context);
    // alterative 2 embedding
    //_imageBlob.deserialize(data,context);
 }

  PinholeImageSensor::PinholeImageSensor(int id, IdContext* context): ImageSensor(id,context){
    _cameraMatrix.setIdentity();
    _distortionModel="";
  }

  PinholeImageSensor::~PinholeImageSensor(){}

  void PinholeImageSensor::serialize(ObjectData& data, IdContext& context){
    ImageSensor::serialize(data, context);
    _cameraMatrix.toBOSS(data, "cameraMatrix");
    data.setString("distortionModel", _distortionModel);
    _distortionParameters.toBOSS(data,"distortionParameters");
  }

  void PinholeImageSensor::deserialize(ObjectData& data, IdContext& context){
    ImageSensor::deserialize(data, context);
    _cameraMatrix.fromBOSS(data, "cameraMatrix");
    _distortionModel=data.getString("distortionModel");
    _distortionParameters.fromBOSS(data, "distortionParameters");
  }

  PinholeImageData::PinholeImageData(PinholeImageSensor* sensor,  int id, IdContext* context):
    ImageData(sensor, id, context){}

  PinholeImageData::~PinholeImageData(){}

  const std::string& PinholeImageData::distortionModel() const {
    return pinholeSensor()->distortionModel();
  }

  void PinholeImageData::setDistortionModel(const std::string& distortionModel_) {
    pinholeSensor()->setDistortionModel(distortionModel_);
  }

  const Eigen::VectorXd& PinholeImageData::distortionParameters() const {
    return pinholeSensor()->distortionParameters();
  }

  void PinholeImageData::setDistortionParameters(const Eigen::VectorXd& distortionParameters_) {pinholeSensor()->setDistortionParameters(distortionParameters_);}

  const Eigen::Matrix3d& PinholeImageData::cameraMatrix() const {return pinholeSensor()->cameraMatrix();}
  void PinholeImageData::setCameraMatrix(const Eigen::Matrix3d& cameraMatrix_) { pinholeSensor()->setCameraMatrix(cameraMatrix_);}

  BOSS_REGISTER_BLOB(ImageBLOB);
  BOSS_REGISTER_CLASS(ImageSensor);
  BOSS_REGISTER_CLASS(ImageData);
  BOSS_REGISTER_CLASS(PinholeImageSensor);
  BOSS_REGISTER_CLASS(PinholeImageData);

}
