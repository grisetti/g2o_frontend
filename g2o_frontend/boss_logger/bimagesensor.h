#ifndef _BOSS_IMAGE_SENSOR_H_
#define _BOSS_IMAGE_SENSOR_H
#include "bsensor.h"
#include "g2o_frontend/boss/blob.h"
#include <Eigen/Core>
#include "opencv2/highgui/highgui.hpp"
namespace boss {
  
  class ImageBLOB: public BLOB {
  public:
    enum Format {mono8=0x0, mono16=0x1, rgb8=0x2};
    ImageBLOB();
    virtual const std::string& extension() const;
    void setExtension(const std::string& extension_) {_extension = extension_;}
    void resize(int width, int height, Format format);
    Format format() const { return _format; }
    virtual bool read(std::istream& is);
    virtual void write(std::ostream& os);
  protected:
    inline cv::Mat& cvImage() {return _image;}
    const cv::Mat& cvImage() const {return _image;}
    std::string _extension;
    cv::Mat _image;
    Format _format;
  };

  typedef BLOBReference<ImageBLOB> ImageBLOBReference;

  class ImageSensor : public BaseSensor {
  public:
    ImageSensor(int id=-1, IdContext* context = 0);
  };

  class Image : public SensorData<ImageSensor>  {
  public:
    enum DistortionModel {PlumbBob=0x0};
    enum CameraModel {Pinhole=0x0, Cylindrical=0x1};
    Image(ImageSensor* sensor=0, 
	  int id=-1, 
	  IdContext* context = 0);
    ~Image();
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    inline ImageBLOBReference& imageBlob() { return _imageBlob; }
    inline const ImageBLOBReference& imageBlob() const { return _imageBlob; }
  protected:
    ImageBLOBReference _imageBlob;
    CameraModel _cameraModel;
    DistortionModel _distortionModel;
    Eigen::MatrixXd _cameraMatrix;
    Eigen::MatrixXd _distortionParameters;
  };

  
}

#endif
