#ifndef _BOSS_IMAGE_SENSOR_H_
#define _BOSS_IMAGE_SENSOR_H_
#include "bsensor.h"
#include "g2o_frontend/boss/blob.h"
#include <Eigen/Core>
#include "opencv2/highgui/highgui.hpp"
namespace boss {
  
  class ImageBLOB: public BLOB {
  public:
    enum Format {mono8=0x0, mono16=0x1, rgb8=0x2};
    ImageBLOB();
    virtual const std::string& extension();
    void setExtension(const std::string& extension_) {_extension = extension_;}
    void resize(int width, int height, Format format);
    Format format() const { return _format; }
    virtual bool read(std::istream& is);
    virtual void write(std::ostream& os);
    inline cv::Mat& cvImage() {return _image;}
    const cv::Mat& cvImage() const {return _image;}
  protected:
    std::string _extension;
    cv::Mat _image;
    Format _format;
  };

  typedef BLOBReference<ImageBLOB> ImageBLOBReference;

  class ImageSensor : public BaseSensor {
  public:
    ImageSensor(int id=-1, IdContext* context = 0);
    virtual ~ImageSensor();
  };

  class ImageData : public SensorData<ImageSensor>  {
  public:
    ImageData(ImageSensor* sensor=0, 
	  int id=-1, 
	  IdContext* context = 0);
    ~ImageData();
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);
    inline ImageBLOBReference& imageBlob() { return _imageBlob; }
    inline const ImageBLOBReference& imageBlob() const { return _imageBlob; }
  protected:
    ImageBLOBReference _imageBlob;
  };

  class PinholeImageSensor: public ImageSensor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PinholeImageSensor(int id=-1, IdContext* context = 0);
    virtual ~PinholeImageSensor();
    virtual void serialize(ObjectData& data, IdContext& context);
    virtual void deserialize(ObjectData& data, IdContext& context);

    inline const std::string& distortionModel() const {return _distortionModel;}
    inline void setDistortionModel(const std::string& distortionModel_) {_distortionModel=distortionModel_;}
    inline const Eigen::VectorXd& distortionParameters() const {return _distortionParameters;}
    inline void setDistortionParameters(const Eigen::VectorXd& distortionParameters_) {_distortionParameters = distortionParameters_;}
    inline const Eigen::Matrix3d& cameraMatrix() const {return _cameraMatrix;}
    inline void setCameraMatrix(const Eigen::Matrix3d& cameraMatrix_) {_cameraMatrix = cameraMatrix_;}
  protected:
    Eigen::Matrix3d _cameraMatrix;
    std::string _distortionModel;
    Eigen::VectorXd _distortionParameters;
  };

  class PinholeImageData: public ImageData {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    PinholeImageData(PinholeImageSensor* sensor=0,  int id=-1, IdContext* context = 0);
    ~PinholeImageData();
    virtual const std::string& distortionModel() const;
    virtual void setDistortionModel(const std::string& distortionModel_);
    virtual const Eigen::VectorXd& distortionParameters() const;
    virtual void setDistortionParameters(const Eigen::VectorXd& distortionParameters_);
    virtual const Eigen::Matrix3d& cameraMatrix() const;
    virtual void setCameraMatrix(const Eigen::Matrix3d& cameraMatrix_);
    inline const PinholeImageSensor* pinholeSensor() const { return dynamic_cast<PinholeImageSensor*>(_sensor); }
    inline PinholeImageSensor* pinholeSensor() { return dynamic_cast<PinholeImageSensor*>(_sensor); }
  };

}

#endif
