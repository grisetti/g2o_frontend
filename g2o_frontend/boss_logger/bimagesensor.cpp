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
    _distortionModel=PlumbBob;
    _cameraModel=Pinhole;
    _cameraMatrix.setIdentity();
    _distortionParameters.resize(1,5);
    _distortionParameters.setZero();
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

    std::string s;
    switch(_distortionModel){
    case PlumbBob: s="PlumbBob"; break;
    default:s="Unknown";
    }
    data.setString("distortionModel", s);
    switch(_cameraModel){
    case Pinhole: s="Pinhole"; break;
    case Cylindrical: s="Cylindrical"; break;
    default:s="Unknown";
    }
    data.setString("cameraModel", s);
    _cameraMatrix.toBOSS(data,"cameraMatrix");
    _distortionParameters.toBOSS(data,"distortionParameters");
  }
  void Image::deserialize(ObjectData& data, IdContext& context){
    cerr << "A";
    SensorData<ImageSensor>::deserialize(data,context);
    // alternative 1, creating an own field
    cerr << "B";
    ObjectData * blobData = static_cast<ObjectData *>(data.getField("imageBlob"));
    _imageBlob.deserialize(*blobData,context);
    // alterative 2 embedding
    //_imageBlob.deserialize(data,context);
     
    cerr << "C";
     std::string s = data.getString("distortionModel");
     if (s=="PlumbBob")
       _distortionModel=PlumbBob;
     else
       _distortionModel=UnknownDistortion;

     s = data.getString("cameraModel");
     if (s=="Pinhole")
       _cameraModel=Pinhole;
     else if (s=="Cylindrical")
       _cameraModel=Cylindrical;
     else 
       _cameraModel=UnknownCamera;
    cerr << "D";
     _cameraMatrix.fromBOSS(data, "cameraMatrix");
    cerr << "E";
     _distortionParameters.fromBOSS(data,"distortionParameters"); 
    cerr << "F";
 }

  BOSS_REGISTER_BLOB(ImageBLOB);
  BOSS_REGISTER_CLASS(Image);
  BOSS_REGISTER_CLASS(ImageSensor);

}
