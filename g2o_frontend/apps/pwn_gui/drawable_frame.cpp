#include "drawable_frame.h"

namespace pwn {

  using namespace g2o;
  using namespace std;
/** this thing keeps the state of the parameters of the viewer. 
    We create the parameters only once and we bound these parameters 
    to each drawable scene we make*/

  DrawableFrameParameters::DrawableFrameParameters(int step){
    float r= 0.3, g = 0.3, b=0.9;
    _pPoints = new GLParameterPoints(1.0f, Vector4f(r, g, b, 1.0f));
    _pPoints->setStep(step);
    _pNormals = new GLParameterNormals(1.0f, Vector4f(0.0f, 0.0f, 1.0f, 1.0f), 0.0f);
    _pNormals->setStep(step);
    _pCovariances = new GLParameterCovariances(1.0f, 
					       Vector4f(0.0f, 1.0f, 0.0f, 1.0f), Vector4f(1.0f, 0.0f, 0.0f, 1.0f),
					       0.02f, 0.0f);
    _pCovariances->setStep(step);
    _pCorrespondences = new GLParameterCorrespondences(1.0f, Vector4f(1.0f, 0.0f, 1.0f, 1.0f), 0.0f);
    _pCorrespondences->setStep(step);
  };

  DrawableFrameParameters::~DrawableFrameParameters(){
    delete _pPoints;
    delete _pCorrespondences;
    delete _pNormals;
    delete _pCovariances;
  }

  void DrawableFrameParameters::applyGLParameter() {}



  DrawableFrame::DrawableFrame(const Eigen::Isometry3f& transformation_, 
			       DrawableFrameParameters* parameters_, 
			       Frame* frame_){
    setTransformation(transformation_);
    _parameters = parameters_;
    _frame = frame_;
    _dPoints=0;
    _dCorrespondences = 0;
    _dCovariances = 0;
    _dCorrespondences = 0;
    _previousDrawableFrame = 0;
    constructDrawableObjects();
  }

  DrawableFrame::~DrawableFrame(){
    clearDrawableObjects();
  }


  void DrawableFrame::setViewer(PWNQGLViewer *viewer_) { 
    Drawable::setViewer(viewer_);
    if (_dPoints){
      _dPoints->setViewer(_viewer);
    }
    if (_dNormals){
      _dNormals->setViewer(_viewer);
    }
    if (_dCovariances){
      _dCovariances->setViewer(_viewer);
    }
    if (_dCorrespondences){
      _dCorrespondences->setViewer(_viewer);
    }

  }


  bool DrawableFrame::setParameter(GLParameter *parameter_) {
    DrawableFrameParameters* dparams = dynamic_cast<DrawableFrameParameters*>(parameter_);
    if (! dparams)
      return false;
    _parameters = dparams;
    return true;
  }

  GLParameter* DrawableFrame::parameter() {return _parameters;}


  void DrawableFrame::clearDrawableObjects(){
    if (_dPoints)
      delete _dPoints;
    if (_dNormals)
      delete _dNormals;
    if (_dCovariances)
      delete _dCovariances;
    if (_dCorrespondences)
      delete _dCorrespondences;
    _dPoints=0;
    _dCorrespondences = 0;
    _dCovariances = 0;
    _dCorrespondences = 0;
  }

  void DrawableFrame::constructDrawableObjects(){
    if (_frame) {
      _dPoints = new DrawablePoints(Isometry3f::Identity(), (GLParameter*)_parameters->_pPoints, &_frame->points(), &_frame->normals());
      _dNormals = new DrawableNormals(Isometry3f::Identity(), (GLParameter*)_parameters->_pNormals, &_frame->points(), &_frame->normals());
      _dCovariances = new DrawableCovariances(Isometry3f::Identity(), (GLParameter*)_parameters->_pCovariances, &_frame->stats());
      //_dCorrespondences = new DrawableCorrespondences(Isometry3f::Identity(), (GLParameter*)_parameters->_pCorrespondences, 0,
      //&frame.points(), &frame.points(), &correspondences);
    }
  }


  void DrawableFrame::setFrame(Frame* f) {
    if (f != _frame){
      clearDrawableObjects();
      _frame = f;
      constructDrawableObjects();
    }
  }

  Frame* DrawableFrame::frame() const {
    return _frame;
  }

  const Eigen::Isometry3f& DrawableFrame::localTransformation() const {return _localTransform;}
  
  void DrawableFrame::setLocalTransformation(const Eigen::Isometry3f& localTransform_) {_localTransform = localTransform_;}

  // Drawing function of the class object.
  void DrawableFrame::draw() {
    if(_parameters->isShown() && _frame){
      glPushMatrix();
      glMultMatrixf(_transformation.data());
      if (_dPoints)
	_dPoints->draw();
      if (_dNormals)
	_dNormals->draw();
      if (_dCovariances)
	_dCovariances->draw();
      if (_dCorrespondences)
	_dCorrespondences->draw();
      glPopMatrix();
    }
  }

}
