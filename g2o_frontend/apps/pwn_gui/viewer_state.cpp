#include "viewer_state.h"

#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/types/slam3d/types_slam3d.h"

#include <unistd.h>

#undef _PWN_USE_CUDA_

#ifdef _PWN_USE_CUDA_
#include "g2o_frontend/pwn_cuda/cualigner.h"
#endif// PWN_CUDA


namespace pwn{

  using namespace std;
  using namespace g2o;


  ViewerState::ViewerState(PWNGuiMainWindow* mwin){
    pwnGMW = mwin;
    graph = 0;
  }

  void ViewerState::init(){
    imageRows = 0;
    imageCols = 0;
    
    ng_worldRadius = 0.1f;
    ng_minImageRadius = 10;
    ng_curvatureThreshold = 1.0f;
    al_innerIterations = 1;
    al_outerIterations = 10;
    vz_step = 5;
    if_curvatureThreshold = 0.1f;
    reduction = 2;


    cameraMatrix <<     
      525.0f, 0.0f, 319.5f,
      0.0f, 525.0f, 239.5f,
      0.0f, 0.0f, 1.0f;
    
    float scale = 1./reduction;
    cameraMatrix*=scale;
    cameraMatrix(2,2) = 1;

    if (0){
      sensorOffset.setIdentity();
    } else {
      sensorOffset.translation() = Vector3f(0.15f, 0.0f, 0.05f);
      Quaternionf quat = Quaternionf(0.5, -0.5, 0.5, -0.5);
      sensorOffset.linear() = quat.toRotationMatrix();
    }

    sensorOffset.matrix().row(3) << 0,0,0,1;

    projector = new PinholePointProjector();
    statsFinder = new StatsFinder();

    pointInformationMatrixFinder = new PointInformationMatrixFinder();
    normalInformationMatrixFinder = new NormalInformationMatrixFinder ;
    converter= new DepthImageConverter (projector, statsFinder, 
					pointInformationMatrixFinder, normalInformationMatrixFinder);

    projector->setTransform(Eigen::Isometry3f::Identity());
    projector->setCameraMatrix(cameraMatrix);

    // Creating and setting aligner object.
    //Aligner aligner;
    correspondenceFinder = new CorrespondenceFinder();
    linearizer = new Linearizer() ;
#ifdef _PWN_USE_CUDA_
    aligner = new CuAligner() ;
#else
    aligner = new Aligner();
#endif
    
    aligner->setProjector(projector);
    aligner->setLinearizer(linearizer);
    linearizer->setAligner(aligner);
    aligner->setCorrespondenceFinder(correspondenceFinder);

    
    statsFinder->setWorldRadius(ng_worldRadius);
    //statsFinder->setCurvatureThreshold(ng_curvatureThreshold);
    statsFinder->setMinPoints(ng_minImageRadius);
    aligner->setInnerIterations(al_innerIterations);
    aligner->setOuterIterations(al_outerIterations);
    converter->_curvatureThreshold = ng_curvatureThreshold;
    pointInformationMatrixFinder->setCurvatureThreshold(if_curvatureThreshold);
    normalInformationMatrixFinder->setCurvatureThreshold(if_curvatureThreshold);

    refScn = pwnGMW->scene0();
    currScn = pwnGMW->scene1();

    listWidget = pwnGMW->listWidget;
    drawableFrameParameters = new DrawableFrameParameters();

    typedef BlockSolver< BlockSolverTraits<-1, -1> >  SlamBlockSolver;
    typedef LinearSolverCSparse<SlamBlockSolver::PoseMatrixType> SlamLinearSolver;
    SlamLinearSolver* linearSolver = new SlamLinearSolver();
    linearSolver->setBlockOrdering(false);
    SlamBlockSolver* blockSolver = new SlamBlockSolver(linearSolver);
    OptimizationAlgorithmGaussNewton* solverGauss = new OptimizationAlgorithmGaussNewton(blockSolver);
    graph = new SparseOptimizer();
    graph->setAlgorithm(solverGauss);
    continuousMode = false;
  }

  void ViewerState::refreshFlags(){
    stepViewer = pwnGMW->step();
    mergePressed = pwnGMW->merge();
    pointsViewer = pwnGMW->points();
    normalsViewer = pwnGMW->normals();
    covariancesViewer = pwnGMW->covariances();
    correspondencesViewer = pwnGMW->correspondences();
    initialGuessViewer = pwnGMW->initialGuess();
    optimizeViewer = pwnGMW->optimize();
    stepByStepViewer = pwnGMW->stepByStep();
    addCloud = pwnGMW->addCloud();
    clearLast = pwnGMW->clearLast();    
    clearAll = pwnGMW->clearAll();
    listItem = listWidget->currentItem();
    if (*mergePressed)
      continuousMode = ! continuousMode;
  }

  void ViewerState::load(const std::string& filename){
    clear();
    listWidget->clear();
    graph->clear();
    graph->load(filename.c_str());

    vector<int> vertexIds(graph->vertices().size());
    int k=0;
    for (OptimizableGraph::VertexIDMap::iterator it = graph->vertices().begin(); it != graph->vertices().end(); ++it) {
      vertexIds[k++] = (it->first);
    }

    sort(vertexIds.begin(), vertexIds.end());

    listAssociations.clear();
    size_t maxCount = 20000;
    for(size_t i = 0; i < vertexIds.size() &&  i< maxCount; ++i) {
      OptimizableGraph::Vertex* _v = graph->vertex(vertexIds[i]);
      g2o::VertexSE3* v = dynamic_cast<g2o::VertexSE3*>(_v);
      if (! v)
	continue;
      OptimizableGraph::Data* d = v->userData();
      while(d) {
	RGBDData* rgbdData = dynamic_cast<RGBDData*>(d);
	if (!rgbdData){
	  d=d->next();
	  continue;
	}
	// retrieve from the rgb data the index of the parameter
	int paramIndex = rgbdData->paramIndex();
	// retrieve from the graph the parameter given the index  
	g2o::Parameter* _cameraParam = graph->parameter(paramIndex);
	// attempt a cast to a parameter camera  
	ParameterCamera* cameraParam = dynamic_cast<ParameterCamera*>(_cameraParam);
	if (! cameraParam){
	  cerr << "shall thou be damned forever" << endl;
	  return;
	}
	// yayyy we got the parameter
	Eigen::Matrix3f cameraMatrix;
	Eigen::Isometry3f sensorOffset;
	cameraMatrix.setZero();
      
	int cmax = 4;
	int rmax = 3;
	for (int c=0; c<cmax; c++){
	  for (int r=0; r<rmax; r++){
	    sensorOffset.matrix()(r,c)= cameraParam->offset()(r, c);
	    if (c<3)
	      cameraMatrix(r,c) = cameraParam->Kcam()(r, c);
	  }
	}
	char buf[1024];
	sprintf(buf,"%d",v->id());
	QString listItem(buf);
	listAssociations.push_back(rgbdData);
	listWidget->addItem(listItem);
	d=d->next();
      }
    }

  }

  void ViewerState::clear(){
    pwnGMW->viewer_3d->clearDrawableList();
    for(size_t i = 0; i < drawableFrameVector.size(); i++){
      if (drawableFrameVector[i]->frame())
	delete drawableFrameVector[i]->frame();
      delete(drawableFrameVector[i]);
    }
    drawableFrameVector.clear();
    globalT = Isometry3f::Identity();
    refScn->clear();
    currScn->clear();
    wasInitialGuess = false;
    newCloudAdded = false;
  }
  
  void ViewerState::addNextAndOptimizeSelected(){
    // advance one row
    if (! listItem)
      return;
    listWidget->setCurrentRow(listWidget->row(listItem) + 1);
    listItem = listWidget->currentItem();
    addCloudSelected();
    optimizeSelected();
    polishOldThings();
    *mergePressed = 0;
  }

  void ViewerState::updateDrawableParameters(){
    // Check feature visualization options.   
    if (stepViewer[0]) {
      drawableFrameParameters->_pPoints->setStep(stepViewer[1]);
      drawableFrameParameters->_pNormals->setStep(stepViewer[1]);
      drawableFrameParameters->_pCovariances->setStep(stepViewer[1]);
      drawableFrameParameters->_pCorrespondences->setStep(stepViewer[1]);
    } else {
      drawableFrameParameters->_pPoints->setStep(1);
      drawableFrameParameters->_pNormals->setStep(1);
      drawableFrameParameters->_pCovariances->setStep(1);
      //drawableFrameParameters->_pCorrespondences->setStep(stepViewer1);
    }

    if(pointsViewer[0])
      drawableFrameParameters->_pPoints->setPointSize(pointsViewer[1]);
    else
      drawableFrameParameters->_pPoints->setPointSize(0.0f);

    if(normalsViewer[0])
      drawableFrameParameters->_pNormals->setNormalLength(normalsViewer[1]);
    else
      drawableFrameParameters->_pNormals->setNormalLength(0.0f);

    if(correspondencesViewer[0])
      drawableFrameParameters->_pCorrespondences->setLineWidth(correspondencesViewer[1]);
    else
      drawableFrameParameters->_pCorrespondences->setLineWidth(0.0f);

    if(covariancesViewer[0])
      drawableFrameParameters->_pCovariances->setEllipsoidScale(covariancesViewer[1]);
    else
      drawableFrameParameters->_pCovariances->setEllipsoidScale(0.0f);
    
    if(correspondencesViewer[0])
      drawableFrameParameters->_pCorrespondences->setLineWidth(correspondencesViewer[1]);
    else
      drawableFrameParameters->_pCorrespondences->setLineWidth(0.0f);
  }

  void ViewerState::initialGuessSelected(){
      cerr << "initial guess" << endl;
      DrawableFrame* current = drawableFrameVector.back();
      DrawableFrame* reference = drawableFrameVector[drawableFrameVector.size()-2];
      current->setTransformation(reference->transformation());
      newCloudAdded = true;
      wasInitialGuess = true;
      *initialGuessViewer = 0;
  }

  bool extractRelativePrior(Eigen::Isometry3f& priorMean, Matrix6f& priorInfo, 
			    const DrawableFrame* reference, const DrawableFrame* current){
    VertexSE3* referenceVertex =reference->_vertex;
    VertexSE3* currentVertex =current->_vertex;
    bool priorFound = false;
    priorInfo.setZero();
    for (HyperGraph::EdgeSet::const_iterator it=referenceVertex->edges().begin();
	 it!= referenceVertex->edges().end(); it++){
      const EdgeSE3* e = dynamic_cast<const EdgeSE3*>(*it);
      if (e->vertex(0)==referenceVertex && e->vertex(1) == currentVertex){
	priorFound=true;
	for (int c=0; c<6; c++)
	  for (int r=0; r<6; r++)
	    priorInfo(r,c) = e->information()(r,c);

	for(int c=0; c<4; c++)
	  for(int r=0; r<3; r++)
	    priorMean.matrix()(r,c) = e->measurement().matrix()(r,c);
	priorMean.matrix().row(3) << 0,0,0,1;
      }
    }
    return priorFound;
  }


  bool extractAbsolutePrior(Eigen::Isometry3f& priorMean, Matrix6f& priorInfo, 
			    const DrawableFrame* current){
    VertexSE3* currentVertex =current->_vertex;
    ImuData* imuData = 0;
    OptimizableGraph::Data* d = currentVertex->userData();
    while(d) {
      ImuData* imuData_ = dynamic_cast<ImuData*>(d);
      if (imuData_){
	imuData = imuData_;
      }
      d=d->next();
    }
	
    if (imuData){
      Eigen::Matrix3d R=imuData->getOrientation().matrix();
      Eigen::Matrix3d Omega = imuData->getOrientationCovariance().inverse();
      priorMean.setIdentity();
      priorInfo.setZero();
      for (int c = 0; c<3; c++)
	for (int r = 0; r<3; r++)
	  priorMean.linear()(r,c)=R(r,c);
      
      for (int c = 0; c<3; c++)
	for (int r = 0; r<3; r++)
	  priorInfo(r+3,c+3)=Omega(r,c);
      return true;
    }
    return false;
  }


  void ViewerState::optimizeSelected(){
      DrawableFrame* current = drawableFrameVector.back();
      DrawableFrame* reference = drawableFrameVector[drawableFrameVector.size()-2];
      cerr << "optimizing" << endl;
      cerr << "current=" << current->frame() << endl;
      cerr << "reference= " << reference->frame() << endl;


      

      // cerr computing initial guess based on the frame positions, just for convenience
      Eigen::Isometry3d delta = reference->_vertex->estimate().inverse()*current->_vertex->estimate();
      for(int c=0; c<4; c++)
	for(int r=0; r<3; r++)
	  initialGuess.matrix()(r,c) = delta.matrix()(r,c);


      Eigen::Isometry3f odometryMean;
      Matrix6f odometryInfo;
      bool hasOdometry = extractRelativePrior(odometryMean, odometryInfo, reference, current);
      if (hasOdometry)
	initialGuess=odometryMean;

      Eigen::Isometry3f imuMean;
      Matrix6f imuInfo;
      bool hasImu = extractAbsolutePrior(imuMean, imuInfo, current);

      initialGuess.matrix().row(3) << 0,0,0,1;
      
      if(!wasInitialGuess) {
	aligner->clearPriors();
	aligner->setOuterIterations(al_outerIterations);
	aligner->setReferenceFrame(reference->frame());
	aligner->setCurrentFrame(current->frame());
	aligner->setInitialGuess(initialGuess);
	aligner->setSensorOffset(sensorOffset);
	if (hasOdometry)
	  aligner->addRelativePrior(odometryMean, odometryInfo);
	if (hasImu)
	  aligner->addAbsolutePrior(reference->transformation(), imuMean, imuInfo);
	aligner->align();
      }

      Eigen::Isometry3f localTransformation =aligner->T();
      if (aligner->inliers()<1000 || aligner->error()/aligner->inliers()>10){
	cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
	cerr << "aligner: monster failure: inliers = " << aligner->inliers() << endl;
	cerr << "aligner: monster failure: error/inliers = " << aligner->error()/aligner->inliers() << endl;
	cerr  << "Local transformation: " << t2v(aligner->T()).transpose() << endl;
      	localTransformation = initialGuess;
	sleep(1);
      }
      cout << "Local transformation: " << t2v(aligner->T()).transpose() << endl;
      
      globalT = reference->transformation()*aligner->T();
      
      // Update cloud drawing position.
      current->setTransformation(globalT);
      current->setLocalTransformation(aligner->T());

      // Show zBuffers.
      refScn->clear();
      currScn->clear();
      QImage refQImage;
      QImage currQImage;
      DepthImageView div;
      div.computeColorMap(300, 2000, 128);
      div.convertToQImage(refQImage, aligner->correspondenceFinder()->referenceDepthImage());
      div.convertToQImage(currQImage, aligner->correspondenceFinder()->currentDepthImage());
      refScn->addPixmap((QPixmap::fromImage(refQImage)).scaled(QSize((int)refQImage.width()/(ng_scale*3), (int)(refQImage.height()/(ng_scale*3)))));
      currScn->addPixmap((QPixmap::fromImage(currQImage)).scaled(QSize((int)currQImage.width()/(ng_scale*3), (int)(currQImage.height()/(ng_scale*3)))));
      pwnGMW->graphicsView1_2d->show();
      pwnGMW->graphicsView2_2d->show();
      
      wasInitialGuess = false;
      newCloudAdded = false;
      *initialGuessViewer = 0;
      *optimizeViewer = 0;
  }

  void ViewerState::addCloudSelected(){
    if(listItem) {
      cerr << "adding a frame" << endl;
      int index = pwnGMW->listWidget->row(listItem);
      cerr << "index: " << endl;
      std::string fname = listAssociations[index]->baseFilename()+"_depth.pgm";
      DepthImage depthImage;
      if (!depthImage.load(fname.c_str(), true)){
      	cerr << " skipping " << fname << endl;
      	newCloudAdded = false;
      	*addCloud = 0;
      	return;
      }
      DepthImage scaledDepthImage;
      DepthImage::scale(scaledDepthImage, depthImage, reduction);
      imageRows = scaledDepthImage.rows();
      imageCols = scaledDepthImage.cols();
      correspondenceFinder->setSize(imageRows, imageCols);
      Frame * frame=new Frame();
      converter->compute(*frame, scaledDepthImage, sensorOffset);
      OptimizableGraph::DataContainer* dc =listAssociations[index]->dataContainer();
      cerr << "datacontainer: " << dc << endl;
      VertexSE3* v = dynamic_cast<VertexSE3*>(dc);
      cerr << "vertex of the frame: " << v->id() << endl;

      DrawableFrame* drawableFrame = new DrawableFrame(globalT, drawableFrameParameters, frame);
      drawableFrame->_vertex = v;
      
      Eigen::Isometry3f priorMean;
      Matrix6f priorInfo;
      bool hasImu =extractAbsolutePrior(priorMean, priorInfo, drawableFrame);
      if (hasImu && drawableFrameVector.size()==0){
        	globalT.linear() = priorMean.linear();
        	cerr << "!!!! i found an imu for the first vertex, me happy" << endl;
        	drawableFrame->setTransformation(globalT);
      }
      drawableFrameVector.push_back(drawableFrame);
      pwnGMW->viewer_3d->addDrawable(drawableFrame);
    }
    newCloudAdded = true;
    *addCloud = 0;
    _meHasNewFrame = true;
  }

  void ViewerState::clearLastSelected(){
    if(drawableFrameVector.size() > 0) {
      pwnGMW->viewer_3d->popBack();
      DrawableFrame* lastDrawableFrame = drawableFrameVector.back();
      if (lastDrawableFrame->frame()){
	       delete lastDrawableFrame->frame();
      }
      delete lastDrawableFrame;
      drawableFrameVector.pop_back();
    }
    if(drawableFrameVector.size() > 0) { 
      DrawableFrame* lastDrawableFrame = drawableFrameVector.back();
      globalT = lastDrawableFrame->transformation();;
    } 
  }

  void ViewerState::polishOldThings(){
    if(drawableFrameVector.size() > 40) {
      pwnGMW->viewer_3d->drawableList().erase(pwnGMW->viewer_3d->drawableList().begin());
      DrawableFrame* firstDrawableFrame = drawableFrameVector.front();
      if (firstDrawableFrame->frame()){
	       delete firstDrawableFrame->frame();
      }
      delete firstDrawableFrame;
      drawableFrameVector.erase(drawableFrameVector.begin());
    }
  }

  void ViewerState::processCommands(){
    _meHasNewFrame = false;
    refreshFlags();
    updateDrawableParameters();
    if(!wasInitialGuess && !newCloudAdded && drawableFrameVector.size() > 1 && *initialGuessViewer) {
      initialGuessSelected();
      continuousMode = false;
    } else if(newCloudAdded && drawableFrameVector.size() > 1 && *optimizeViewer && !(*stepByStepViewer)) {
      optimizeSelected();
      continuousMode = false;
    }
    // Add cloud was pressed.
    else if(*addCloud) {
      addCloudSelected();
      continuousMode = false;
    }
    // clear buttons pressed.
    else if(*clearAll) {
      clear();
      *clearAll = 0;
      continuousMode = false;
    }
    else if(*clearLast) {
      clearLastSelected();
      continuousMode = false;
    } 
    else if(continuousMode){
      addNextAndOptimizeSelected();
    }
    // To avoid memorized commands to be managed.
    *initialGuessViewer = 0;
    *optimizeViewer = 0;
    *addCloud = 0; 
    *clearAll = 0;
    *clearLast = 0;

    if (0 && drawableFrameVector.size()){
      Eigen::Isometry3f globalT = drawableFrameVector.front()->transformation();
      qglviewer::Vec robotPose(globalT.translation().x(), globalT.translation().y(), globalT.translation().z());
      qglviewer::Vec robotAxisX(globalT.linear()(0,0), globalT.linear()(1,0), globalT.linear()(2,0));
      qglviewer::Vec robotAxisZ(globalT.linear()(0,2), globalT.linear()(1,2), globalT.linear()(2,2));
      pwnGMW->viewer_3d->camera()->setPosition(robotPose+.5*robotAxisZ+.5*robotAxisX);
      pwnGMW->viewer_3d->camera()->setUpVector(robotAxisX+robotAxisZ);
      pwnGMW->viewer_3d->camera()->setViewDirection(robotPose+.5*robotAxisZ+.5*robotAxisX);
    }


    pwnGMW->viewer_3d->updateGL();
  }

} 
