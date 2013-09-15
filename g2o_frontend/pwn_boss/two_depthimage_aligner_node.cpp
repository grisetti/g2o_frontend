#include "two_depthimage_aligner_node.h"


namespace pwn_boss {
  using namespace pwn;
  using namespace boss;
  using namespace boss_map;
  using namespace boss_logger;



  template <typename T1, typename T2>
  void convertScalar(T1& dest, const T2& src){
    for (int i=0; i<src.matrix().cols(); i++)
      for (int j=0; j<src.matrix().rows(); j++)
	dest.matrix()(j,i) = src.matrix()(j,i);

  }

  void computeScaledParameters (int& rows, int& cols, Eigen::Matrix3f& cameraMatrix, float scale) {
    cameraMatrix.block<2,3>(0,0)*=1./scale;
    rows *=1./scale;
    cols *=1./scale;
  }


  TwoDepthImageAlignerNode::TwoDepthImageAlignerNode(MapManager* manager_,
						     RobotConfiguration* config_,
						     DepthImageConverter* converter_,
						     Aligner* aligner_,
						     const std::string& topic_): MapNodeProcessor(manager_,config_){
    _previousSensingFrameNode = 0;
    _previousFrame = 0;
    _topic = topic_;
    _converter = converter_;
    _aligner = aligner_;
    _scale = 4;
    _counter = 0;
    
  }

  void TwoDepthImageAlignerNode::processNode(MapNode* node_){
    SensingFrameNode* sensingFrame = dynamic_cast<SensingFrameNode*>(node_);
    if (! sensingFrame)
      return;
    
    PinholeImageData* image = dynamic_cast<PinholeImageData*>(sensingFrame->sensorData(_topic));
    if (! image)
      return;
    cerr << "got image"  << endl;
    
    Eigen::Isometry3d _sensorOffset = _config->sensorOffset(image->baseSensor());
    
    // cerr << "sensorOffset: " << endl;
    // cerr << _sensorOffset.matrix() << endl;

    Eigen::Isometry3f sensorOffset;
    convertScalar(sensorOffset,_sensorOffset);
    sensorOffset.matrix().row(3) << 0,0,0,1;

    Eigen::Matrix3d _cameraMatrix = image->cameraMatrix();
    
    ImageBLOB* blob = image->imageBlob().get();
    
    DepthImage depthImage;
    depthImage.fromCvMat(blob->cvImage());
    int r=depthImage.rows();
    int c=depthImage.cols();
    
    DepthImage scaledImage;
    DepthImage::scale(scaledImage,depthImage,_scale);
    Eigen::Matrix3f cameraMatrix;
    convertScalar(cameraMatrix,_cameraMatrix);
    
    computeScaledParameters(r,c,cameraMatrix,_scale);
    PinholePointProjector* projector=dynamic_cast<PinholePointProjector*>(_converter->_projector);
    cameraMatrix(2,2)=1;
    projector->setCameraMatrix(cameraMatrix);
    pwn::Frame* frame = new pwn::Frame;
    _converter->compute(*frame,scaledImage, sensorOffset);

 
    MapNodeBinaryRelation* odom=0;

    std::vector<MapNode*> oneNode(1);
    oneNode[0]=sensingFrame;
    MapNodeUnaryRelation* imu = extractRelation<MapNodeUnaryRelation>(oneNode);
    
    if (_previousFrame){
      _aligner->setReferenceSensorOffset(_aligner->currentSensorOffset());
      _aligner->setCurrentSensorOffset(sensorOffset);
      _aligner->setReferenceFrame(_previousFrame);
      _aligner->setCurrentFrame(frame);
      
      _aligner->correspondenceFinder()->setSize(r,c);
      PinholePointProjector* projector=(PinholePointProjector*)(_aligner->projector());
      projector->setCameraMatrix(cameraMatrix);

      /*
	cerr << "correspondenceFinder: "  << r << " " << c << endl; 
	cerr << "sensorOffset" << endl;
	cerr <<_aligner->currentSensorOffset().matrix() << endl;
	cerr <<_aligner->referenceSensorOffset().matrix() << endl;
	cerr << "cameraMatrix" << endl;
	cerr << projector->cameraMatrix() << endl;
      */

      std::vector<MapNode*> twoNodes(2);
      twoNodes[0]=_previousSensingFrameNode;
      twoNodes[1]=sensingFrame;
      odom = extractRelation<MapNodeBinaryRelation>(twoNodes);
      cerr << "odom:" << odom << " imu:" << imu << endl;

      Eigen::Isometry3f guess= Eigen::Isometry3f::Identity();
      _aligner->clearPriors();
      if (odom){
      	Eigen::Isometry3f mean;
      	Eigen::Matrix<float,6,6> info;
      	convertScalar(mean,odom->transform());
	mean.matrix().row(3) << 0,0,0,1;
	convertScalar(info,odom->informationMatrix());
	cerr << "odom: " << t2v(mean).transpose() << endl;
	_aligner->addRelativePrior(mean,info);
 	//guess = mean;
      } 

      if (imu){
      	Eigen::Isometry3f mean;
      	Eigen::Matrix<float,6,6> info;
      	convertScalar(mean,imu->transform());
      	convertScalar(info,imu->informationMatrix());
	mean.matrix().row(3) << 0,0,0,1;
	cerr << "imu: " << t2v(mean).transpose() << endl;
	_aligner->addAbsolutePrior(_globalT,mean,info);
      }
      _aligner->setInitialGuess(guess);
      cerr << "Frames: " << _previousFrame << " " << frame << endl;

      
      // projector->setCameraMatrix(cameraMatrix);
      // projector->setTransform(Eigen::Isometry3f::Identity());
      // Eigen::MatrixXi debugIndices(r,c);
      // DepthImage debugImage(r,c);
      // projector->project(debugIndices, debugImage, frame->points());

      _aligner->align();
      
      // sprintf(buf, "img-dbg-%05d.pgm",j);
      // debugImage.save(buf);
      //sprintf(buf, "img-ref-%05d.pgm",j);
      //_aligner->correspondenceFinder()->referenceDepthImage().save(buf);
      //sprintf(buf, "img-cur-%05d.pgm",j);
      //_aligner->correspondenceFinder()->currentDepthImage().save(buf);

      cerr << "inliers: " << _aligner->inliers() << endl;
      cerr << "chi2: " << _aligner->error() << endl;
      cerr << "chi2/inliers: " << _aligner->error()/_aligner->inliers() << endl;
      cerr << "initialGuess: " << t2v(guess).transpose() << endl;
      cerr << "transform   : " << t2v(_aligner->T()).transpose() << endl;
      if (_aligner->inliers()>-1){
 	_globalT = _globalT*_aligner->T();
	cerr << "TRANSFORM FOUND" <<  endl;
      } else {
	cerr << "FAILURE" <<  endl;
	_globalT = _globalT*guess;
      }
      if (! (_counter%50) ) {
	Eigen::Matrix3f R = _globalT.linear();
	Eigen::Matrix3f E = R.transpose() * R;
	E.diagonal().array() -= 1;
	_globalT.linear() -= 0.5 * R * E;
      }
      _globalT.matrix().row(3) << 0,0,0,1;
      cerr << "globalTransform   : " << t2v(_globalT).transpose() << endl;

      char buf[1024];
      sprintf(buf, "frame-%05d.pwn",_counter);
      frame->save(buf, 1, true, _globalT);


      *os << _globalT.translation().transpose() << endl;
    } else {
      _aligner->setCurrentSensorOffset(sensorOffset);
      _globalT = Eigen::Isometry3f::Identity();
      if (imu){
      	Eigen::Isometry3f mean;
      	convertScalar(mean,imu->transform());
	_globalT = mean;
      }
    }
    
   
    if (_previousFrame)
      delete _previousFrame;
    
    delete blob;

    _previousSensingFrameNode = sensingFrame;
    _previousFrame = frame;
    _counter++;
  }

}
