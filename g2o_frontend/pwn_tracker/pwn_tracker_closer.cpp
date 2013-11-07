
#include "g2o_frontend/boss_logger/bframe.h"
#include "g2o_frontend/pwn2/frame.h"
#include "g2o_frontend/pwn2/pinholepointprojector.h"
#include "g2o_frontend/pwn2/depthimageconverter.h"
#include "g2o_frontend/pwn2/aligner.h"
#include "g2o_frontend/boss/serializer.h"
#include "g2o_frontend/boss/deserializer.h"
#include "g2o/types/slam3d/vertex_se3.h"
#include "g2o/types/slam3d/edge_se3.h"

#include "highgui.h"
#include <fstream>
#include <iostream>
#include "pwn_tracker.h"
using namespace std;
using namespace pwn;
using namespace boss;
using namespace pwn_tracker;


  
struct PwnTrackerClosureRelation: public PwnTrackerRelation {
  PwnTrackerClosureRelation(MapManager* manager=0, int id=-1, IdContext* context = 0):
    PwnTrackerRelation(manager, id, context){
    reprojectionInliers = 0;
    reprojectionOutliers = 0;
  }

  virtual void serialize(ObjectData& data, IdContext& context){
    PwnTrackerRelation::serialize(data,context);
    data.setInt("reprojectionInliers", reprojectionInliers);
    data.setInt("reprojectionOutliers", reprojectionOutliers);
  }

  virtual void deserialize(ObjectData& data, IdContext& context){
    PwnTrackerRelation::deserialize(data,context);
    reprojectionInliers = data.getInt("reprojectionInliers");
    reprojectionOutliers = data.getInt("reprojectionOutliers");
  }

  int reprojectionInliers;
  int reprojectionOutliers;
};

BOSS_REGISTER_CLASS(PwnTrackerClosureRelation);

  float compareNormals(cv::Mat& m1, cv::Mat& m2){
    if (m1.rows!=m2.rows || m1.cols != m2.cols)
      return 0;
    return norm(abs(m1-m2));
  }

  float compareDepths(cv::Mat& m1, cv::Mat& m2){
    if (m1.rows!=m2.rows || m1.cols != m2.cols)
      return 0;
    // consides only the pixels that are >0;
    cv::Mat mask = (m1>0) & (m2>0);
    //cerr << "mask!" << endl;

    mask.convertTo(mask, m1.type());
    //cerr << "convert" << endl;

    cv::Mat diff = (m1-m2)&mask;

    //cerr << "diffMask" << endl;

    diff.convertTo(diff, CV_32FC1);
    return norm(diff);
  }

struct MatchingCandidate{
  int fromIdx;
  int toIdx;
  PwnTrackerClosureRelation* relation;
  float normalDifference;
  float depthDifference;
  float reprojectionDistance;
  int outliers;
  int inliers;
  cv::Mat  diffRegistered;
  cv::Mat  normalsRegistered;
};

Frame* makeFrame(DepthImageConverter* converter, PwnTrackerFrame* trackerFrame, int scale = 8){
  boss_logger::ImageBLOB* depthBLOB = trackerFrame->depthImage.get();
  PinholePointProjector* projector = (PinholePointProjector*)converter->_projector;
  projector->setImageSize(trackerFrame->imageRows, trackerFrame->imageCols);
  pwn::DepthImage depth;
  pwn::Frame* cloud=new pwn::Frame;
  Eigen::Matrix3f cameraMatrix;
  Eigen::Isometry3f offset;
  depth.fromCvMat(depthBLOB->cvImage());
  convertScalar(offset, trackerFrame->sensorOffset);
  convertScalar(cameraMatrix, trackerFrame->cameraMatrix);
  projector->setCameraMatrix(cameraMatrix);
  pwn::DepthImage scaledDepth;
  DepthImage::scale(scaledDepth, depth, scale);
  projector->scale (1./scale);
  converter->compute(*cloud, scaledDepth, offset);
  delete depthBLOB;
  return cloud;
}


struct FrameCluster{
  int clusterNumber;
  std::vector<PwnTrackerFrame*>  trackerFrames;
  std::map<PwnTrackerFrame*,pwn::Frame*> pwnFrames;
  std::vector<MatchingCandidate*> matchingCandidates;

  void makeClouds(DepthImageConverter* converter,
		  int scale = 8){
    cerr << "   frames: ";
    for (size_t i = 0; i<trackerFrames.size(); i++){
      Frame* f = makeFrame(converter, trackerFrames[i], scale);
      pwnFrames.insert(make_pair(trackerFrames[i],f));
      cerr << trackerFrames[i] << "(" <<  f->points().size() << ") ";
      // char filename[1024]; sprintf(filename, "frame-%05d.pwn",i);
      // f->save(filename, 1,  true);
    }
    cerr << endl;
  }

  void clearClouds() {
    for (std::map<PwnTrackerFrame*,pwn::Frame*>::iterator it = pwnFrames.begin(); it!=pwnFrames.end(); it++){
      delete it->second;
    }
    pwnFrames.clear();
  } 

  void matchCorrespondence(Aligner* aligner, MatchingCandidate& candidate,
			   std::vector<PwnTrackerFrame*>& trackerFrames,
			   MapManager* manager, int scale = 8){
    PwnTrackerFrame* from = trackerFrames[candidate.fromIdx];
    PwnTrackerFrame* to = trackerFrames[candidate.toIdx];
    if (from == to)
      return; 
   
    cerr <<  " matching frames: " << from << " " << to << endl;
    
    pwn::Frame* fromCloud, *toCloud; 
    toCloud = this->pwnFrames[to];
    fromCloud = this->pwnFrames[from];

    cerr << "retrieved clouds: " << fromCloud << " " << toCloud << endl;

    Eigen::Isometry3f fromOffset, toOffset;
    Eigen::Matrix3f fromCameraMatrix, toCameraMatrix;

    convertScalar(fromOffset, from->sensorOffset);
    convertScalar(fromCameraMatrix, from->cameraMatrix);

    convertScalar(toOffset, to->sensorOffset);
    convertScalar(toCameraMatrix, to->cameraMatrix);
    
    PinholePointProjector* projector = (PinholePointProjector*)aligner->projector();
    int r, c;
  
    aligner->setReferenceSensorOffset(fromOffset);
    aligner->setCurrentSensorOffset(toOffset);
    aligner->setInitialGuess(Eigen::Isometry3f::Identity());
    projector->setCameraMatrix(toCameraMatrix);
    projector->setImageSize(to->imageRows,to->imageCols);
    projector->scale(1./scale);
    
    cerr << "cameraMatrix: " << endl;
    cerr << projector->cameraMatrix() << endl;
    r = projector->imageRows();
    c = projector->imageCols();
    cerr << "r: " << r << ", c: " << c << endl;

    aligner->correspondenceFinder()->setImageSize(r,c);
    aligner->setReferenceFrame(fromCloud);
    aligner->setCurrentFrame(toCloud);
    aligner->align();
    
    Eigen::Isometry3d relationMean;
    convertScalar(relationMean, aligner->T());
    PwnTrackerClosureRelation* rel = new PwnTrackerClosureRelation(manager);
    rel->setTransform(relationMean);
    rel->setInformationMatrix(Eigen::Matrix<double, 6,6>::Identity());
    rel->setTo(from);
    rel->setFrom(to);
    candidate.relation = rel;

    DepthImage 
      currentDepthThumb = aligner->correspondenceFinder()->currentDepthImage(),
      referenceDepthThumb = aligner->correspondenceFinder()->referenceDepthImage();
    cv::Mat currentRect, referenceRect;
    currentDepthThumb.toCvMat(currentRect);
    referenceDepthThumb.toCvMat(referenceRect);
    cv::Mat mask = (currentRect>0) & (referenceRect>0);
    currentRect.convertTo(currentRect, CV_32FC1);
    referenceRect.convertTo(referenceRect, CV_32FC1);
    mask.convertTo(mask, currentRect.type());
    cv::Mat diff = abs(currentRect-referenceRect)&mask;
    candidate.diffRegistered = diff.clone();
    int nonZeros = countNonZero(mask);
    float sum=0;
    for (int i = 0; i<diff.rows; i++)
      for (int j = 0; j<diff.cols; j++){
	float d = candidate.diffRegistered.at<float>(i,j);
	sum +=d;
      }
    candidate.relation = rel;
    candidate.reprojectionDistance = sum/nonZeros;
  }

  void matchCorrespondences(Aligner* aligner, std::vector<PwnTrackerFrame*>& pwnFrames,
			    MapManager* manager, int scale = 8){
    for(size_t i=0; i<matchingCandidates.size(); i++){
      matchCorrespondence(aligner, *matchingCandidates[i], pwnFrames, manager, scale);
    }
  }
};


cv::Mat normalsComparison;
cv::Mat depthsComparison;
std::vector<boss::Serializable*> objects;
std::vector<PwnTrackerFrame*> trackerFrames;
std::vector<PwnTrackerRelation*> trackerRelations;
std::vector<cv::Mat> normalImages;
std::vector<cv::Mat> depthImages;
std::vector<MatchingCandidate> matchingCandidates;
int r = 0;
int c = 0;
float a = 0.0;
float cutoff = 0;
float inlierThreshold = 50;
int maxOutliers = 100;

MapManager* load(std::vector<PwnTrackerFrame*>& trackerFrames,
		 std::vector<PwnTrackerRelation*>& trackerRelations,
		 std::vector<Serializable*>& objects,
		 Deserializer& des){
  
  Serializable* o=0;
  boss_map::MapManager* manager = 0;
  while(o=des.readObject()){
    objects.push_back(o);
    boss_map::MapManager* m = dynamic_cast<boss_map::MapManager*>(o);
    if (m) {
      manager = m;
    }
    PwnTrackerFrame* f = dynamic_cast<PwnTrackerFrame*>(o);
    if (f) {
      trackerFrames.push_back(f);
    }
    PwnTrackerRelation* r = dynamic_cast<PwnTrackerRelation*>(o);
    if (r) {
      trackerRelations.push_back(r);
    }
  }
  cerr << "read " << objects.size() << " elements";
  cerr << "manager: " << manager << endl;
  cerr << "# frames: " << trackerFrames.size() << endl;
  cerr << "# relations: " << trackerRelations.size() << endl;
  return manager;
}

void save(std::vector<Serializable*>& objects,
	  MapManager* manager,
	  std::vector<MatchingCandidate>& candidates,
	  Serializer& ser){
  for (size_t i = 0; i<objects.size(); i++)
    ser.writeObject(*objects[i]);
  for(size_t i = 0; i<candidates.size(); i++){
    MatchingCandidate& c = candidates[i];
    ser.writeObject(*c.relation);
  }
}


void g2oSave(std::vector<PwnTrackerFrame*>& trackerFrames, 
	     std::vector<PwnTrackerRelation*>& trackerRelations,
	     std::vector<MatchingCandidate> candidates,
	     int maxOutliers){
  
  std::map<PwnTrackerFrame*, g2o::VertexSE3*> vertexMap;
  g2o::OptimizableGraph graph;

  for (size_t i=0; i<trackerFrames.size(); i++){
    PwnTrackerFrame* f = trackerFrames[i];
    g2o::VertexSE3* v = new g2o::VertexSE3;
    v->setId(i);
    v->setEstimate(f->transform());
    graph.addVertex(v);
    vertexMap.insert(make_pair(f,v));
  }

  for (size_t i = 0; i<trackerRelations.size(); i++){
    PwnTrackerRelation* rel = trackerRelations[i];
    g2o::EdgeSE3* e = new g2o::EdgeSE3;
    g2o::VertexSE3* vFrom = vertexMap[(PwnTrackerFrame*)rel->nodes()[0]];
    g2o::VertexSE3* vTo = vertexMap[(PwnTrackerFrame*)rel->nodes()[1]];
    e->setVertex(0,vFrom);
    e->setVertex(1,vTo);
    e->setMeasurement(rel->transform());
    e->setInformation(rel->informationMatrix()*1000);
    graph.addEdge(e);
  }

  for(size_t i = 0; i<candidates.size(); i++){
    MatchingCandidate& c = candidates[i];
    if (c.outliers<maxOutliers){
      PwnTrackerRelation* rel = c.relation;
      g2o::VertexSE3* vFrom = vertexMap[(PwnTrackerFrame*)rel->nodes()[0]];
      g2o::VertexSE3* vTo = vertexMap[(PwnTrackerFrame*)rel->nodes()[1]];
      // Eigen::Isometry3d delta=vFrom->estimate().inverse()*vTo->estimate()*rel->transform().inverse();
      // AngleAxisd aa(delta.linear());
      // if (aa.angle()>M_PI/2 || delta.translation().norm()>1)
      // 	continue;
      g2o::EdgeSE3* e = new g2o::EdgeSE3;
      e->setVertex(0,vFrom);
      e->setVertex(1,vTo);
      e->setMeasurement(rel->transform());
      e->setInformation(rel->informationMatrix());
      graph.addEdge(e);
      char filename[1024];
      sprintf(filename, "match-%05d-%05d.pgm", vFrom->id(), vTo->id());
      cv::imwrite(filename, c.diffRegistered);
    }
  }
  graph.save("out.g2o");
}

void fetchNormalThumbnails(std::vector<cv::Mat>& normalImages, 
			   std::vector<PwnTrackerFrame*>& trackerFrames,
			   Deserializer& des){
  // load the images
  for(size_t i = 0; i<trackerFrames.size(); i++){
    cv::Mat normalImg;
    PwnTrackerFrame *n = trackerFrames[i];
    ImageBLOB* thumbnail = n->normalThumbnail.get();
    thumbnail->cvImage().convertTo(normalImg, CV_32FC3);
    normalImg=normalImg-127.0f;
    normalImg=normalImg*(1./255);
    normalImages.push_back(normalImg);
    delete thumbnail;
  }
  cerr << "done normals" << endl;
}

void fetchDepthThumbnails(std::vector<cv::Mat>& depthImages, 
			  std::vector<PwnTrackerFrame*>& trackerFrames,
			  Deserializer& des){
  for(size_t i = 0; i<trackerFrames.size(); i++){
    PwnTrackerFrame *n = trackerFrames[i];
    ImageBLOB* thumbnail = n->depthThumbnail.get();
    cv::Mat depthImg;
    thumbnail->cvImage().convertTo(depthImg,CV_32FC1);
    depthImg = depthImg *(1./65535);
    depthImages.push_back(depthImg);
    delete thumbnail;
  }
  cerr << "done depths" << endl;
}

cv::Mat generateDepthComparison(std::vector<PwnTrackerFrame*>& trackerFrames,
				std::vector<cv::Mat>& depthImages){
  int numImages = trackerFrames.size();
  cv::Mat depthsComparison(numImages, numImages, CV_32FC1);
  float maxDepthDifference = 0;
  for (int i = 0; i<depthsComparison.rows; i++) {
    depthsComparison.at<float>(i,i)=0;
    for (int j = 0; j<i; j++){
      cv::Mat& m1  = depthImages[i];
      cv::Mat& m2  = depthImages[j];
      float f = compareDepths(m1,m2);
      //cerr << "val: " << i << " " << j << " " << f << endl;
      depthsComparison.at<float>(j,i)=f;
      depthsComparison.at<float>(i,j)=f;
      if (maxDepthDifference<f)
	maxDepthDifference = f;
    }
  }
  cerr << "maxDepthDifference: " << maxDepthDifference << endl;
  maxDepthDifference = 1./maxDepthDifference;
  for (int i = 0; i<depthsComparison.rows; i++)
    for (int j = 0; j<depthsComparison.rows; j++){
      depthsComparison.at<float>(i,j)*=maxDepthDifference;
    }
  return depthsComparison;
}

cv::Mat generateNormalComparison(std::vector<PwnTrackerFrame*>& trackerFrames,
				  std::vector<cv::Mat>& normalImages){
  int numImages = trackerFrames.size();
  cv::Mat normalsComparison(numImages, numImages, CV_32FC1);
  float maxNormalDifference = 0;
  for (int i = 0; i<normalsComparison.rows; i++) {
    normalsComparison.at<float>(i,i)=0;
    for (int j = 0; j<i; j++){
      cv::Mat& m1  = normalImages[i];
      cv::Mat& m2  = normalImages[j];
      float f = compareNormals(m1,m2);
      //cerr << "val: " << i << " " << j << " " << f << endl;
      normalsComparison.at<float>(j,i)=f;
      normalsComparison.at<float>(i,j)=f;
      if (maxNormalDifference<f)
	maxNormalDifference = f;
    }
  }
  cerr << "maxNormalDifference: " << maxNormalDifference << endl;
  maxNormalDifference = 1./maxNormalDifference;
  for (int i = 0; i<normalsComparison.rows; i++)
    for (int j = 0; j<normalsComparison.rows; j++){
      normalsComparison.at<float>(i,j)*=maxNormalDifference;
    }
  return normalsComparison;
}

void constructCandidates(std::vector<MatchingCandidate>& candidates, 
			 cv::Mat depthComparison, float depthThreshold,
			 cv::Mat normalsComparison, float normalsThreshold){
  int r = depthComparison.rows;
  //int c = depthComparison.cols;
  for (int i=0; i<r; i++)
    for (int j=0; j<i; j++){
      float d = depthComparison.at<float>(i,j);
      float n = normalsComparison.at<float>(i,j);
      if (d<depthThreshold && n < normalsThreshold){
	MatchingCandidate mc;
	mc.fromIdx = i;
	mc.toIdx = j;
	mc.relation  = 0;
	mc.normalDifference = n;
	mc.depthDifference = d;
	candidates.push_back(mc);
      }
    }
}




void alignFrames(Aligner* aligner, DepthImageConverter* converter,
		 MatchingCandidate& candidate,
		 std::vector<PwnTrackerFrame*>& trackerFrames,
		 MapManager* manager){
  PwnTrackerFrame* from = trackerFrames[candidate.fromIdx];
  PwnTrackerFrame* to = trackerFrames[candidate.toIdx];
  if (from == to)
    return;

  //pwn::Frame* fromCloud = from->cloud.get();
  //pwn::Frame* toCloud = to->cloud.get();
  boss_logger::ImageBLOB* fromDepthBlob = from->depthImage.get();
  boss_logger::ImageBLOB* toDepthBlob = to->depthImage.get();
  
  PinholePointProjector* projector = (PinholePointProjector*)converter->_projector;
  pwn::DepthImage fromDepth, toDepth;
  pwn::Frame fromCloud, toCloud; 
  Eigen::Isometry3f fromOffset, toOffset;
  Eigen::Matrix3f fromCameraMatrix, toCameraMatrix;
  int scale = 8;
  int r, c;
  
  {
    fromDepth.fromCvMat(fromDepthBlob->cvImage());
    convertScalar(fromOffset, from->sensorOffset);
    convertScalar(fromCameraMatrix, from->cameraMatrix);
    projector->setImageSize(from->imageRows, from->imageCols);
    projector->setCameraMatrix(fromCameraMatrix);
    pwn::DepthImage scaledFromDepth;
    DepthImage::scale(scaledFromDepth, fromDepth, scale);
    projector->scale (1./scale);
    converter->compute(fromCloud, scaledFromDepth, fromOffset);
  }

  {
    toDepth.fromCvMat(toDepthBlob->cvImage());
    convertScalar(toOffset, to->sensorOffset);
    convertScalar(toCameraMatrix, to->cameraMatrix);
    projector->setCameraMatrix(toCameraMatrix);
    pwn::DepthImage scaledToDepth;
    projector->setImageSize(from->imageRows, from->imageCols);
    DepthImage::scale(scaledToDepth, toDepth, scale);
    projector->scale (1./scale);
    converter->compute(toCloud, scaledToDepth, toOffset);
    r = projector->imageRows();
    c = projector->imageCols();
  }
  
  PinholePointProjector* alignerProjector = (PinholePointProjector*)aligner->projector();
  aligner->setReferenceSensorOffset(fromOffset);
  aligner->setCurrentSensorOffset(toOffset);
  aligner->setInitialGuess(Eigen::Isometry3f::Identity());
  alignerProjector->setImageSize(r,c);
  alignerProjector->setCameraMatrix(projector->cameraMatrix());
  aligner->correspondenceFinder()->setImageSize(r,c);
  aligner->setReferenceFrame(&fromCloud);
  aligner->setCurrentFrame(&toCloud);
  aligner->align();

  Eigen::Isometry3d relationMean;
  convertScalar(relationMean, aligner->T());
  PwnTrackerClosureRelation* rel = new PwnTrackerClosureRelation(manager);
  rel->setTransform(relationMean);
  rel->setInformationMatrix(Eigen::Matrix<double, 6,6>::Identity());
  rel->setTo(from);
  rel->setFrom(to);
  candidate.relation = rel;
  delete toDepthBlob;
  delete fromDepthBlob;

  DepthImage currentDepthThumb, referenceDepthThumb;
  DepthImage::scale(currentDepthThumb, aligner->correspondenceFinder()->currentDepthImage(),1);
  DepthImage::scale(referenceDepthThumb, aligner->correspondenceFinder()->referenceDepthImage(),1);
  cv::Mat currentRect, referenceRect;
  currentDepthThumb.toCvMat(currentRect);
  referenceDepthThumb.toCvMat(referenceRect);
  cv::Mat mask = (currentRect>0) & (referenceRect>0);
  currentRect.convertTo(currentRect, CV_32FC1);
  referenceRect.convertTo(referenceRect, CV_32FC1);
  mask.convertTo(mask, currentRect.type());
  cv::Mat diff = abs(currentRect-referenceRect)&mask;
  candidate.diffRegistered = diff.clone();
  int nonZeros = countNonZero(mask);
  float sum=0;
  for (int i = 0; i<diff.rows; i++)
    for (int j = 0; j<diff.cols; j++){
      float d = candidate.diffRegistered.at<float>(i,j);
      sum +=d;
    }
  candidate.relation = rel;
  candidate.reprojectionDistance = sum/nonZeros;
}


int findRoot(std::vector<int>& assignment, int idx){
  if (assignment[idx]==-1 || assignment[idx] == idx){
    assignment[idx] = idx;
    return idx;
  }
  int r = findRoot(assignment, assignment[idx]); 
  assignment[idx] = r;
  return r;
}

void partition(std::vector<int>& parents, 
	       std::vector<PwnTrackerFrame*>& trackerFrames, 
	       std::vector<MatchingCandidate>&  matchingCandidates) {
  parents.resize(trackerFrames.size(), -1);

  for (size_t i = 0; i< trackerFrames.size(); i++)
    parents[i] = -1;

  for (size_t i =0; i<matchingCandidates.size(); i++) {
    const MatchingCandidate& m=matchingCandidates[i];
    int aIdx = std::min(m.fromIdx, m.toIdx);
    int bIdx = std::max(m.fromIdx, m.toIdx);
    int aRoot = findRoot(parents, aIdx);
    int bRoot = findRoot(parents, bIdx);
    if (aRoot < bRoot ){
      parents[bRoot] = aRoot;
    } else {
      parents[aRoot] = bRoot;
    }
  }
  //cerr << "Constructed tree" << endl;
  for (size_t i=0; i<parents.size(); i++) {
    parents[i] = findRoot(parents, i);
    //cerr << "node: " << i << "cluster:" << parents[i] << endl;
  }
}

void makeClusters(std::map<int, FrameCluster >& clusters,
		  std::vector<PwnTrackerFrame*>& trackerFrames, 
		  std::vector<MatchingCandidate>&  matchingCandidates) {
  std::vector<int> partitions;
  partition(partitions, trackerFrames, matchingCandidates); 

  for(size_t i=0; i<partitions.size(); i++) {
    std::map<int, FrameCluster >::iterator it=clusters.find(partitions[i]);
    if (it == clusters.end()) {
      FrameCluster cluster;
      cluster.clusterNumber = partitions[i];
      cluster.trackerFrames.push_back(trackerFrames[i]);
      clusters.insert(make_pair(cluster.clusterNumber, cluster));
    } else { 
      it->second.trackerFrames.push_back(trackerFrames[i]);
    }
  }

  for (size_t i =0; i<matchingCandidates.size(); i++) {
    MatchingCandidate& m = matchingCandidates[i];
    int cnum1 = partitions[m.fromIdx];
    int cnum2 = partitions[m.toIdx];
    if (cnum1 != cnum2) {
      cerr << "ERROR: " 
	   << matchingCandidates[i].fromIdx << ":" << cnum1 << " "
	   << matchingCandidates[i].toIdx   << ":" << cnum2 << endl;
      throw runtime_error("correspondences mismatch");
    }
    clusters[cnum1].matchingCandidates.push_back(&matchingCandidates[i]);
  }
}

void scoreCandidate(MatchingCandidate& candidate, float inlierThreshold){
  cv::Mat& diff = candidate.diffRegistered;
  int inliers = 0;
  int outliers = 0;
  //int region = 3;
  for (int i = 0; i<diff.rows; i++)
    for (int j = 0; j<diff.cols; j++){
      // int dmin = 2*inlierThreshold;
      // for (int k=-region; k<region; k++)
      // 	for (int l=-region; l<region; l++){
      // 	  int a = i+k;
      // 	  int b = j+l;
      // 	  if (a<0 || a>diff.rows)
      // 	    continue;
      // 	  if (b<0 || b>diff.rows)
      // 	    continue;
      // 	  float d = diff.at<float>(i,j);
      // 	  if (d>0 && dmin>d)
      // 	    dmin = d;
      // 	}
	  
      float d = diff.at<float>(i,j);
      if (d < inlierThreshold)
	inliers ++;
      else
	outliers++;
    }
  candidate.inliers = inliers;
  candidate.outliers = outliers;
  candidate.relation->reprojectionInliers=inliers;
  candidate.relation->reprojectionOutliers=outliers;
}


void autoMatch2(Aligner* aligner, DepthImageConverter* converter,
		std::map<int, FrameCluster>& clusters,
		std::vector<PwnTrackerFrame*>& trackerFrames,
		MapManager* manager, int scale = 8){
  for (std::map<int, FrameCluster>::iterator it = clusters.begin(); it!=clusters.end(); it++){
    FrameCluster& cluster = it->second;
    if (cluster.matchingCandidates.size()<1)
      continue;
    cerr << "processing cluster: " << it->first << endl;
    cerr << "  size: " << it->second.trackerFrames.size() << endl;
    cerr << "  computing clouds...  ";
    cluster.makeClouds(converter, scale);
    cerr << "Done" << endl;
    
    cerr << "  computing alignments (" << cluster.matchingCandidates.size() << ") ... ";
    cluster.matchCorrespondences(aligner, trackerFrames, manager, scale);
    cerr << "Done" << endl;

    cluster.clearClouds();
  }
}

void scoreCandidates(std::vector<MatchingCandidate>& candidates, float inlierThreshold){
  for (size_t i=0; i<candidates.size(); i++){
    scoreCandidate(candidates[i], inlierThreshold);
    cerr << "scoring frame " << i << "/" << candidates.size() 
	 << " indices: " << candidates[i].fromIdx << "," << candidates[i].toIdx
	 << " rd: " << candidates[i].reprojectionDistance 
	 << " inliers: " << candidates[i].inliers  
	 << " outliers: " << candidates[i].outliers << endl; 
  }
}

void mouseEvent(int evt, int x, int y, int flags, void* param){
    if(evt==CV_EVENT_LBUTTONDOWN){
      r = x;
      c = y;
      cerr << "r,c:" << r << " " << c << endl;
    }
}

int main(int argc, char** argv){
  if (argc<4) {
    cerr << " u should provide a config file and an input file" << endl;
    return 0;
  }

  Aligner* aligner;
  DepthImageConverter* converter;
  std::vector<Serializable*> instances = readConfig(aligner, converter, argv[1]);
  cerr << "config loaded" << endl;
  cerr << " aligner:" << aligner << endl;
  cerr << " converter:" << converter << endl;

  Deserializer des;
  Serializer  ser;
  des.setFilePath(argv[2]);
  // load the log
  MapManager* manager = load(trackerFrames, trackerRelations, objects, des);

  cerr << "generating fingerprints" << endl;
  fetchDepthThumbnails(depthImages, trackerFrames, des);
  fetchNormalThumbnails(normalImages, trackerFrames, des);

  cerr << "generating depth distance matrix" << endl;
  depthsComparison = generateDepthComparison(trackerFrames, depthImages);

  cerr << "generating normal distance matrix" << endl;
  normalsComparison = generateNormalComparison(trackerFrames, normalImages);


  cvNamedWindow("similarity", 0); 
  cvNamedWindow("cutoff", 0); 
  cvNamedWindow("normals1", 0);
  cvNamedWindow("normals2", 0);
  cvNamedWindow("normalsDiff", 0);
  cvNamedWindow("depths1", 0);
  cvNamedWindow("depths2", 0);
  cvNamedWindow("depthsDiff", 0);
  cvNamedWindow("rectDiff", 0);
  cvSetMouseCallback("cutoff", mouseEvent, 0);
  //cvSetMouseCallback("depthsComparison", mouseEvent, 0);
  
  cv::Mat regDiff;
      
  int numImages = trackerFrames.size();
  int ca = 0;
  std::map<int, FrameCluster> clusters;
  while(ca!=27){
    cv::Mat mix = depthsComparison*a + normalsComparison*(1-a);
    cv::Mat shownImage = mix>cutoff;
    int nz = numImages*numImages-countNonZero(shownImage)-numImages;
    cv::circle(shownImage, cv::Point(r,c), 4, 0.0f);
    cv::imshow("cutoff", shownImage);
    cv::imshow("similarity", mix);
    ca = cv::waitKey();
    switch (ca) {
    case 1113938: c --; break;
    case 1113940: c ++; break;
    case 1113937: r --; break;
    case 1113939: r ++; break;
    case 1114027: cutoff += 0.01; break;
    case 1114029: cutoff -= 0.01; break;
    case 1179563: maxOutliers += 1; break;
    case 1179565: maxOutliers -= 1; break;
    case 1048673: a+=0.01; break; // 'a'
    case 1048674: a-=0.01; break; // 'b'
    case 1048608:
      {// ' space '
      MatchingCandidate cand;
      cand.fromIdx = r;
      cand.toIdx = c;
      alignFrames(aligner, converter,
		  cand, trackerFrames, manager);
      regDiff=cand.diffRegistered*(1000.0f/65535.0f);
      scoreCandidate(cand, inlierThreshold);
      cerr << " indices: " << cand.fromIdx << "," << cand.toIdx
	   << " rd: " << cand.reprojectionDistance 
	   << " inliers: " << cand.inliers  
	   << " outliers: " << cand.outliers 
	   << " dmax: " << *std::max_element(cand.diffRegistered.begin<float>(),cand.diffRegistered.end<float>()) <<  endl; 
      cv::imshow("rectDiff", regDiff);
      
      }
      break;
    case 1048675: // 'c'
      matchingCandidates.clear();
      constructCandidates(matchingCandidates, 
			  depthsComparison, 1, 
			  normalsComparison, cutoff);
      cerr << "found " << matchingCandidates.size() << " matching candidates" << endl;
      {
	std::vector<int> assignments;
	partition(assignments, trackerFrames, matchingCandidates);
	clusters.clear();
	makeClusters(clusters, trackerFrames, matchingCandidates);
	for (std::map<int, FrameCluster>::iterator it=clusters.begin(); it!=clusters.end(); it++){
	  
	  FrameCluster& cluster = it->second;
	  if (cluster.trackerFrames.size()>1) {
	    cerr << "partition " << cluster.clusterNumber << "[ " << cluster.trackerFrames.size() << " ] : ";
	    for (size_t i=0; i<cluster.trackerFrames.size(); i++){
	      cerr << cluster.trackerFrames[i] << " ";
	    }
	    cerr << endl;
	  }
	}
	cerr << "number of non zero partitions: " << clusters.size() << endl;
      }
      break;
    case 1048676: // 'd'
      autoMatch2(aligner, converter,
		 clusters,
		 trackerFrames,
		 manager,4);
	break;
    case 1048677: // 'e'
      scoreCandidates(matchingCandidates, inlierThreshold);
      break;
    case 1048678: // 'f'
      cerr << "writing g2o" << endl;
      //ser.setFilePath(argv[3]);
      //save(objects,manager,matchingCandidates,ser,200);
      g2oSave(trackerFrames,trackerRelations,matchingCandidates,maxOutliers);
      break;
    case 1048679: // 'g'
      cerr << "writing boss with closures" << endl;
      ser.setFilePath(argv[3]);
      save(objects,manager,matchingCandidates,ser);
    default: std::cerr << ca << endl;
    }
    if (r>=numImages)
      r = numImages-1;
    if (c>=numImages)
      c = numImages-1;
    if (r<0)
      r = 0;
    if (c<0)
      c = 0;
    cv::Mat& m1 = normalImages[r];
    cv::Mat& m2 = normalImages[c];
    printf("frames: %d %d, a:  %f, cutoff: %f, maxOutliers %d, nz: %d, normalDifference: %f, depthDifference: %f \n",
	   r,c, 
	   a, cutoff, maxOutliers, nz,
	   normalsComparison.at<float>(c,r), 
	   depthsComparison.at<float>(c,r));
    cv::imshow("normals1", m1);
    cv::imshow("normals2", m2);
    cv::Mat diff = abs(m1-m2);
    cv::imshow("normalsDiff", diff);

    cv::imshow("depths1", depthImages[r]);
    cv::imshow("depths2", depthImages[c]);
    cv::imshow("depthsDiff", 100*abs(depthImages[r]-depthImages[c]));
  }

  return 0; 
  
}
