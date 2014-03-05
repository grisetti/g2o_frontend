#include "merger2.h"

using namespace std;

namespace pwn_tracker {
  using namespace boss;
	using namespace std;
  using namespace pwn;

  Merger2::Merger2(int id, boss::IdContext* context): Identifiable(id, context) {
    _depthImageConverter = 0;
	_scale=1;

	 _r=0;
	_c=0;

	_depth_tot=DepthImage::zeros(_r,_c);
	_image_points_count=0;
	_image_overlapping_points_count=0;
	_image_pesi=DepthImage::zeros(_r,_c);
	_bigCloud=new Cloud;
	_currentBigCloud=new Cloud;
	_cloud_tot=new Cloud;
	sts=new Stats();

  }

  Merger2::~Merger2(){}

	void Merger2::clearCloud(){
		_cloud_tot->clear();
		_pesi_tot.clear();
		
	}

	void Merger2::clear(){
		//_cloud_tot->clear();
		//_pesi_tot.clear();
		//_depth_tot=DepthImage::zeros(_r,_c);
		_image_points_count=0;
		_image_overlapping_points_count=0;
		_image_pesi=DepthImage::zeros(_r,_c);
	}

	void Merger2::init(){
		_scale=1;
		_cloud_tot=new pwn::Cloud;
		PinholePointProjector* pointProjector=dynamic_cast<PinholePointProjector*>(_depthImageConverter->projector());
		//PointProjector *pointProjector = _depthImageConverter->projector();
		assert(pointProjector);

		//std::cout<<pointProjector->_cameraMatrix<<std::endl;
		float invScale = 1.0f / _scale;
		Matrix3f scaledCameraMatrix = pointProjector->cameraMatrix() * invScale;
		scaledCameraMatrix(2, 2) = 1.0f;

		pointProjector->setCameraMatrix(scaledCameraMatrix);
		int scaledImageRows = pointProjector->imageRows() / _scale;
		int scaledImageCols = pointProjector->imageCols()*2 / _scale;
		pointProjector->setImageSize(scaledImageRows, scaledImageCols);
	}

	void Merger2::scale(DepthImage& dest, const DepthImage& src, int step){
	  int rows = src.rows/step;
	  int cols = src.cols/step;
	 dest = DepthImage(rows,cols,0.0f);
	  for (int c = 1; c<dest.cols-1; c++){
	    for (int r = 1; r<dest.rows-1; r++){
	        if(src(r*step,c*step)>.2)
			dest(r,c) = src(r*step,c*step);
	    }
	  }
	}

  void Merger2::mergeDepthImage(DepthImage& out,const DepthImage& scaledImage_current) {

	_image_overlapping_points_count=0;
	for (int i=0;i<scaledImage_current.rows;i++){
		for (int j=0;j<scaledImage_current.cols;j++){
			if(scaledImage_current(i,j)>0.1&&scaledImage_current(i,j)<10000){
				_image_overlapping_points_count++;
				float peso=1/(scaledImage_current(i,j));
				float peso_current=peso;
				if(out(i,j)==0||((scaledImage_current(i,j)-out(i,j))<-.00003)){
					//if(_depth_tot(i,j)==0)
						_image_points_count++;
					
					out(i,j)=scaledImage_current(i,j);
					_image_pesi(i,j)=peso_current;
				}else{
					if(fabs(scaledImage_current(i,j)-out(i,j))<.2){
						float peso_tot=_image_pesi(i,j);
						float somma_pesi=peso_tot+peso_current;
						out(i,j)=(out(i,j)*peso_tot+scaledImage_current(i,j)*peso_current)/somma_pesi;
						_image_pesi(i,j)=somma_pesi;
					}
				}
			}
		}
	}
  }




	void Merger2::merge(Eigen::Isometry3f& transform, Eigen::Isometry3f& offset, CloudWithImageSize* cloud) {

	PinholePointProjector* pointProjector=dynamic_cast<PinholePointProjector*>(_depthImageConverter->projector());
	assert(pointProjector);

	DepthImage scaledImage_current;
	IntImage idx_current;
	pointProjector->setTransform(offset);
	pointProjector->project(idx_current, 
		    scaledImage_current, 
		    cloud->points());

	IntImage indexImage_tot=-IntImage::ones(pointProjector->imageRows(), pointProjector->imageCols());
	DepthImage depthImage_tot=DepthImage::zeros(pointProjector->imageRows(), pointProjector->imageCols());

    	pointProjector->setTransform(transform*offset);
	//std::cout<<"cloud tot size "<<_cloud_tot->points().size()<<std::endl;
	if(_cloud_tot->points().size()>0){
		pointProjector->project(indexImage_tot, 
			    depthImage_tot, 
			    _cloud_tot->points());
	}

	for (int i=0;i<scaledImage_current.rows;i++){
		for (int j=0;j<scaledImage_current.cols;j++){
			if(scaledImage_current(i,j)>0.2&&scaledImage_current(i,j)<100){
				float peso=1/(scaledImage_current(i,j));
				float peso_current=peso;
				if(indexImage_tot(i,j)<0){
					//Normal n;
						
						_cloud_tot->points().push_back(transform*cloud->points()[idx_current(i,j)]);
						_cloud_tot->normals().push_back(transform*cloud->normals()[idx_current(i,j)]);
						sts->setN(cloud->stats()[idx_current(i,j)].n());
						sts->setEigenValues(cloud->stats()[idx_current(i,j)].eigenValues());
						sts->setCurvature(cloud->stats()[idx_current(i,j)].curvature());
						sts->block<4, 4>(0, 0)=transform*sts->block<4, 4>(0, 0);
						_cloud_tot->stats().push_back(*sts);
						_cloud_tot->pointInformationMatrix().push_back(cloud->pointInformationMatrix()[idx_current(i,j)].transform(transform.matrix()));
						_cloud_tot->normalInformationMatrix().push_back(cloud->normalInformationMatrix()[idx_current(i,j)].transform(transform.matrix()));
						Gaussian3f g=cloud->gaussians()[idx_current(i,j)];
						_cloud_tot->gaussians().push_back(Gaussian3f(transform.rotation()*g.mean()+transform.translation(),transform.rotation() * g.covarianceMatrix() * transform.rotation().transpose(),false));
						_pesi_tot.push_back(peso);
					
				}else{
					//Normal n;
					Point p=transform*cloud->points()[idx_current(i,j)];
					if(fabs(scaledImage_current(i,j)-depthImage_tot(i,j))<.15){
						if(pointProjector->unProject(p,j,i,scaledImage_current(i,j))){
							int index=indexImage_tot(i,j);
							float peso_tot=_pesi_tot[index];
							float somma_pesi=peso_tot+peso_current;
							_cloud_tot->points()[index][0]=(_cloud_tot->points()[index][0]*peso_tot+p[0]*peso_current)/somma_pesi;
							_cloud_tot->points()[index][1]=(_cloud_tot->points()[index][1]*peso_tot+p[1]*peso_current)/somma_pesi;
							_cloud_tot->points()[index][2]=(_cloud_tot->points()[index][2]*peso_tot+p[2]*peso_current)/somma_pesi;
							_pesi_tot[index]=somma_pesi;
						}
					}else{
						if(scaledImage_current(i,j)-depthImage_tot(i,j)<-.3){
							_cloud_tot->points().push_back(transform*cloud->points()[idx_current(i,j)]);
							_cloud_tot->normals().push_back(transform*cloud->normals()[idx_current(i,j)]);
							sts->setN(cloud->stats()[idx_current(i,j)].n());
							sts->setEigenValues(cloud->stats()[idx_current(i,j)].eigenValues());
							sts->setCurvature(cloud->stats()[idx_current(i,j)].curvature());
							sts->block<4, 4>(0, 0)=transform*sts->block<4, 4>(0, 0);
							_cloud_tot->stats().push_back(*sts);
							_cloud_tot->pointInformationMatrix().push_back(cloud->pointInformationMatrix()[idx_current(i,j)].transform(transform.matrix()));
							_cloud_tot->normalInformationMatrix().push_back(cloud->normalInformationMatrix()[idx_current(i,j)].transform(transform.matrix()));
							Gaussian3f g=cloud->gaussians()[idx_current(i,j)];
							_cloud_tot->gaussians().push_back(Gaussian3f(transform.rotation()*g.mean()+transform.translation(),transform.rotation() * g.covarianceMatrix() * transform.rotation().transpose(),false));
							_pesi_tot.push_back(peso);
						}
					}
				}
			}
		}
	}
  }

	void Merger2::matchWithPartition(DepthImage& currentPartitionImage, Eigen::Isometry3f& offset, DepthImage& partitionMerged){
		PinholePointProjector* pp=dynamic_cast<PinholePointProjector*>(_depthImageConverter->projector());

		//int r,c;
		//Eigen::Matrix3f K=pp->cameraMatrix();
		//pwn::Cloud* bigCloud=_matcher->makeCloud(r,c,K,offset, partitionMerged);
		_bigCloud->clear();
		_currentBigCloud->clear();
		_depthImageConverter->compute(*_bigCloud, partitionMerged, offset);
		_depthImageConverter->compute(*_currentBigCloud, currentPartitionImage, offset);
		
		Eigen::Isometry3d ig=Eigen::Isometry3d::Identity();
		_matcher->clearPriors();
		_matcher->matchClouds(_result, 
				  _currentBigCloud, _bigCloud, 
				  offset, offset,
				  pp->cameraMatrix(), pp->imageRows(), pp->imageCols(), 
				  ig);
	}

  void Merger2::serialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::serialize(data,context);
	 _bconverter=dynamic_cast<pwn_boss::DepthImageConverter*>(_depthImageConverter);
	data.setPointer("converter", _bconverter);
	 data.setPointer("matcher", _matcher);
  }
  
  void Merger2::deserialize(boss::ObjectData& data, boss::IdContext& context){
    Identifiable::deserialize(data,context);
	data.getReference("converter").bind(_depthImageConverter);
	data.getReference("matcher").bind(_matcher);
	_r=_depthImageConverter->projector()->imageRows();
	_c=_depthImageConverter->projector()->imageCols();
  }

  BOSS_REGISTER_CLASS(Merger2);

}
