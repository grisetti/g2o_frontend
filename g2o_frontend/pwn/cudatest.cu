#include "cudatest.h"
#include <cstdio>
#include <fstream>
using namespace std;

//#define __device__ inline
//#define __host__ inline


__device__ void vecFill(float* v, float x, int n){
  for(int i=0; i<n; i++)
    v[i]=x;
}

__device__ void vecScale(float* v, float s, int n){
  for(int i=0; i<n; i++)
    v[i]*=s;
}

__device__ void vecCopy(float* dest, const float* src, int n){
  for(int i=0; i<n; i++)
    dest[i]=src[i];
}

__device__ void vecSum(float*dest, const float* src, float scale, int n){
  for(int i=0; i<n; i++)
    dest[i]+=scale*src[i];
}

__device__ float vecDot(const float* v1, float* v2, int n){
  float a=0;
  for(int i=0; i<n; i++)
    a+=v1[i]*v2[i];
  return a;
}

__device__ void matVecMul(float* dest, const float* A, const float*b, int rows, int cols){
  vecFill(dest, 0, rows);
  for (int i=0; i<cols; i++){
    vecSum(dest,A,b[i],rows);
    A+=rows;
  }
}

__device__ void matMatMul(float* dest, const float* A, const float*B, int ra, int ca, int cb){
  float* dptr = dest;
  const float* bptr = B;
  for (int i=0; i<cb; i++){
    matVecMul(dptr,A,bptr,ra,ca);
    dptr += ra;
    bptr += ca;
  }
}


__device__ void matTranspose(float* dest, const float* src, int rows, int cols){
  for (int i=0; i<cols; i++)
    for (int j=0; j<rows; j++)
      dest[j*cols+i] = src[i*rows+j];
}


template <int n>
__device__ void vecFill(float* v, float x){
  for(int i=0; i<n; i++)
    v[i]=x;
}

template <int n>
__device__ void vecScale(float* v, float s){
  for(int i=0; i<n; i++)
    v[i]*=s;
}

template <int n>
__device__ void vecCopy(float* dest, const float* src){
  for(int i=0; i<n; i++)
    dest[i]=src[i];
}

template <int n>
__device__ void vecSum(float*dest, const float* src, float scale){
  for(int i=0; i<n; i++)
    dest[i]+=scale*src[i];
}

template <int n>
__device__ float vecDot(const float* v1, const float* v2){
  float a=0;
  for(int i=0; i<n; i++)
    a+=v1[i]*v2[i];
  return a;
}

template <int rows, int cols>
__device__ void matVecMul(float* dest, const float* A, const float*b){
  vecFill<rows>(dest, 0);
  for (int i=0; i<cols; i++){
    vecSum<rows>(dest,A,b[i]);
    A+=rows;
  }
}

template <int ra, int ca, int cb>
__device__ void matMatMul(float* dest, const float* A, const float*B){
  float* dptr = dest;
  const float* bptr = B;
  for (int i=0; i<cb; i++){
    matVecMul<ra,ca>(dptr,A,bptr);
    dptr += ra;
    bptr += ca;
  }
}


template <int rows, int cols>
__device__ void matTranspose(float* dest, const float* src){
  for (int i=0; i<cols; i++)
    for (int j=0; j<rows; j++)
      dest[j*cols+i] = src[i*rows+j];
}

__device__ void matBuildSkew(float* m, const float* v){
  const float x = 2*v[0];
  const float y = 2*v[1];
  const float z = 2*v[2];
  m[0] =  0;   m[4] =  z;  m[8]  = -y; m[12] = 0;   
  m[1] = -z;   m[5] =  0;  m[9]  =  x; m[13] = 0;   
  m[2] =  y;   m[6] = -x;  m[10] =  0; m[14] = 0;   
  m[3] =  0;   m[7] =  0;  m[11] =  0; m[15] = 0;   
}

__device__ void transformInverse(float* d, const float* s) {
  d[0] =  s[0];   d[4] =  s[1];  d[8]  = s[2];  d[12] = 0;   
  d[1] =  s[4];   d[5] =  s[5];  d[9]  = s[6];  d[13] = 0;   
  d[2] =  s[8];   d[6] =  s[9];  d[10] = s[10]; d[14] = 0;   
  d[3] =  0;      d[7] =    0;   d[11] =  0; d[15] = 1;
  float t[4];
  matVecMul<4,4>(t,d,s+12);
  d[12] = -t[0];
  d[13] = -t[1];
  d[14] = -t[2];
}

__device__ void _v2t(float* m, const float* v) {
  const float& tx = v[0];
  const float& ty = v[1];
  const float& tz = v[2];
  const float& qx = v[3];
  const float& qy = v[4];
  const float& qz = v[5];
  const float qw = sqrt(1.f - vecDot<3>(v+3,v+3));
  float _m[] = {qw*qw + qx*qx - qy*qy - qz*qz,  2*(qx*qy + qz*qw),              2*(qx*qz - qy*qw),              0, 
		2*(qx*qy - qw*qz) ,             qw*qw - qx*qx + qy*qy - qz*qz,  2*(qy*qz + qx*qw),               0,
		2*(qx*qz + qw*qy),              2*(qy*qz - qx*qw),              qw*qw - qx*qx - qy*qy + qz*qz,  0,
		tx,                             ty,                             tz,                             1
  };
  vecCopy<16>(m,_m);
}

__device__ void _t2v(float* v, const float* m) {
  const float& m00 = m[0];
  const float& m10 = m[1];
  const float& m20 = m[2];
  //const float& m30 = m[3];
  const float& m01 = m[4];
  const float& m11 = m[5];
  const float& m21 = m[6];
  //const float& m31 = m[7];
  const float& m02 = m[8];
  const float& m12 = m[9];
  const float& m22 = m[10];
  //const float& m32 = m[11];
  v[0]  = m[12];
  v[1]  = m[13];
  v[2]  = m[14];
  float tr = m00 + m11 + m22;
  float qx, qy, qz, qw;
  if (tr > 0) { 
    float S = sqrt(tr+1.0) * 2; // S=4*qw 
    qw = 0.25 * S;
    qx = (m21 - m12) / S;
    qy = (m02 - m20) / S; 
    qz = (m10 - m01) / S; 
  } else if ((m00 > m11)&(m00 > m22)) { 
    float S = sqrt(1.0 + m00 - m11 - m22) * 2; // S=4*qx 
    qw = (m21 - m12) / S;
    qx = 0.25 * S;
    qy = (m01 + m10) / S; 
    qz = (m02 + m20) / S; 
  } else if (m11 > m22) { 
    float S = sqrt(1.0 + m11 - m00 - m22) * 2; // S=4*qy
    qw = (m02 - m20) / S;
    qx = (m01 + m10) / S; 
    qy = 0.25 * S;
    qz = (m12 + m21) / S; 
  } else { 
    float S = sqrt(1.0 + m22 - m00 - m11) * 2; // S=4*qz
    qw = (m10 - m01) / S;
    qx = (m02 + m20) / S;
    qy = (m12 + m21) / S;
    qz = 0.25 * S;
  }
  v[3] = qx;
  v[4] = qy;
  v[5] = qz;
}




__device__ void Aligner_projectPoint(const AlignerContext* context, const float* point){
  float ip[4];
  matVecMul<4,4>(ip, context->KT, point);
  float iw=1./ip[2];
  int x = iw*ip[0];
  int y = iw*ip[1];
  int d = 1000*ip[2];
  int pixelPos =context->referenceDepths.rows*y + x;
  if (d<0 || d > context->maxDepth || pixelPos > context->referenceDepths.rows*context->referenceDepths.cols)
    return;
  atomicMin((context->referenceDepths.values+pixelPos),d);
}



__device__ void IntMatrix_map(IntMatrix* img, int rows, int cols, int* points){
  img->rows=rows;
  img->cols=cols;
  img->values=points;
};

__device__  int IntMatrix_getPointAt(IntMatrix* img, int r, int c) {
  return img->values[c*img->rows+r];
}

__device__ void IntMatrix_setPointAt(IntMatrix* img, int r, int c, int v) {
  img->values[c*img->rows+r]=v;
}



template <int rows>
__device__ void FloatMatrix_map(FloatMatrix<rows>* m, int /*_rows*/, int cols, float* values){
  //m->rows=rows;
  m->cols=cols;
  m->values=values;
}

template <int rows>
__device__ const float* FloatMatrix_getColumnAt(const FloatMatrix<rows>* m, int i){
  return m->values+(i*rows);
}

template <int rows>
__device__ void FloatMatrix_setColumnAt(FloatMatrix<rows>* m, int i, const float* src){
  float* dest = m->values+(i*m->rows);
  vecCopy(dest, src, m->rows);
}


__device__ int Aligner_processCorrespondence(float* error,
					     float* Htt,
					     float* Hrr,
					     float* Htr,
					     float* bt,
					     float* br,
					     int referenceIndex, int currentIndex,
					     const AlignerContext* context,
					     int* noNormal,
					     int* tooDistant,
					     int* badCurvature){

  const float* currentPoint    =  FloatMatrix_getColumnAt(&context->currentPoints, currentIndex);
  const float* currentNormal   =  FloatMatrix_getColumnAt(&context->currentNormals, currentIndex);
  const float* omegaP          =  FloatMatrix_getColumnAt(&context->currentOmegaPs, currentIndex);
  const float* omegaN          =  FloatMatrix_getColumnAt(&context->currentOmegaNs, currentIndex);
  float   currentCurvature     =  context->currentCurvatures[currentIndex];
  const float* referencePoint_  =  FloatMatrix_getColumnAt(&context->referencePoints, referenceIndex);
  const float* referenceNormal_ =  FloatMatrix_getColumnAt(&context->referenceNormals, referenceIndex);
  float referenceCurvature     =  context->referenceCurvatures[referenceIndex];
  


  float referencePoint[4];
  float referenceNormal[4];
  float pointsDifference[4];

  const float* T=context->transform;


  // float _trp[4], _trn[4];
  // vecCopy<4>(_trp,referencePoint_);
  // vecCopy<4>(_trn,referenceNormal_);
  //_trp[3]=1.0f;
  //_trn[3]=0.0f;


  matVecMul<4,4>(referencePoint,T,referencePoint_);
  matVecMul<4,4>(referenceNormal,T,referenceNormal_);

  referenceCurvature = (referenceCurvature<context->flatCurvatureThreshold)?context->flatCurvatureThreshold:referenceCurvature;

  currentCurvature = (currentCurvature<context->flatCurvatureThreshold)?context->flatCurvatureThreshold:currentCurvature; 
  float curvatureRatio=(referenceCurvature + 1e-5)/(currentCurvature + 1e-5);
  float normalsRatio = vecDot<4>(currentNormal,referenceNormal);
  
  vecCopy<4>(pointsDifference,referencePoint);
  vecSum<4>(pointsDifference,currentPoint,-1.0f);
  float pointsDistance=vecDot<4>(pointsDifference,pointsDifference);

  if (normalsRatio < context->normalThreshold){
    (*noNormal)++;
    return 0;
  }
  if (pointsDistance > context->distanceThreshold){
    (*tooDistant)++;
    return 0;
  }
  if ((curvatureRatio < context->minCurvatureRatio) ||
      (curvatureRatio > context->maxCurvatureRatio)){
    (*badCurvature)++;
    return 0;
  }
    
    

  int increment = 
    (normalsRatio > context->normalThreshold) &
    (pointsDistance < context->distanceThreshold) &
    (curvatureRatio > context->minCurvatureRatio) &
    (curvatureRatio < context->maxCurvatureRatio);
  if (! increment)  
    return 0;
  
  float normalsDifference[4];
  vecCopy<4>(normalsDifference,referenceNormal);
  vecSum<4>(normalsDifference,currentNormal,-1.0f);
  

  //const Vector4f ep = omegaP*pointError;
  float ep[4];
  matVecMul<4,4>(ep, omegaP, pointsDifference);

  //const Vector4f en = omegaN*normalError;
  float en[4];
  matVecMul<4,4>(en, omegaN, normalsDifference);

  //matPrint(ep,4,1,"ep");
  //matPrint(en,4,1,"en");

  //float localError = pointError.dot(ep) + normalError.dot(en);
  
  float localError = vecDot<4>(ep,pointsDifference) + vecDot<4>(en,normalsDifference);

  int chiOk = localError < context->inlierThreshold;
  //if (! chiOk)     return 0;
  float scale = chiOk * increment; // scale is = 0 if we are in front of an inlier

  
  //Matrix4f Sp = skew(referencePoint);
  float Sp[16];
  matBuildSkew(Sp,referencePoint);
  // compute the transposed (for the skew is mult by -1)

  //Matrix4f Sn = skew(referenceNormal);
  float Sn[16];
  matBuildSkew(Sn,referenceNormal);


  //Htt = omegaP;
  vecCopy<16>(Htt,omegaP);
  
  // prepare to undo if necessary
  vecScale<16>(Htt,scale);

  //Htr.noalias() = omegaP*Sp;
  matMatMul<4,4,4>(Htr,omegaP,Sp);

  //Hrr.noalias() = - (Sp*omegaP*Sp + Sn*omegaN*Sn);
  float temp[16], temp2[16];
  matMatMul<4,4,4>(Hrr,Sp,Htr);
  matMatMul<4,4,4>(temp,Sn,omegaN);
  matMatMul<4,4,4>(temp2,temp,Sn);
  vecSum<16>(Hrr,temp2,+1.0f);
  vecScale<16>(Hrr,-1.0f);
  //bt.noalias() = ep;

  vecCopy<4>(bt,ep);


  //br.noalias() = - (Sp*ep + Sn*en);
  matVecMul<4,4>(br,Sp,ep);
  matVecMul<4,4>(temp,Sn,en);
  vecSum<4>(br,temp,+1.0f);
  vecScale<4>(br,-1.0f);

  // vecScale<4>(bt,scale);
  // vecScale<4>(br,scale);
  // vecScale<4>(Htt,scale);
  // vecScale<4>(Htr,scale);
  // vecScale<4>(Hrr,scale);

  *error = localError;
  return scale*chiOk;
}


#ifdef REAL_CUDA_THING
// src is the base of the shared data of the kernel;
__device__ void ReduceMatrix_shared(float* blockResult, int tid, int bdim, int bidx){
  // contiguous range pattern
  extern __shared__ float sdata[];
  for(int offset = bdim / 2;  offset > 0; offset >>= 1)
    {
      if(tid < offset)
	{
	  // add a partial sum upstream to our own
	  vecSum<56>(sdata +(tid * 56), sdata +( (tid + offset) * 56), +1);
	}
      
      // wait until all threads in the block have
      // updated their partial sums
      __syncthreads();
    }
  if (tid == 0)
    vecCopy<56>(blockResult + (bidx*56),sdata);
  __syncthreads();
}

__global__ void ReduceMatrix_global(float* blockResult, float* input, int n){
  extern __shared__ float sdata[];
  //unsigned int i = bdim * bidx + tid;
  float x[56];
  vecFill<56>(x,0);
  unsigned int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i<n){
    vecCopy<56>(x,input+(i*56));
  }
  vecCopy<56>(sdata+(threadIdx.x*56),x);
  ReduceMatrix_shared(blockResult, threadIdx.x, blockDim.x, blockIdx.x);
}


__global__ void Aligner_1stKernel(AlignerContext* context){
  extern __shared__ float sdata[];
  //unsigned int i = bdim * bidx + tid;
  float x[56];
  vecFill<56>(x,0);
  int i = blockDim.x * blockIdx.x + threadIdx.x;
 
  int retval = 0;
  float error;
  float Htt[16];
  float Hrr[16];
  float Htr[16];
  float bt[4];
  float br[4];
  if (i < context->currentIndices.rows*context->currentIndices.cols) {
    vecCopy<56>(sdata+(threadIdx.x*56),x);
    int referenceIndex=context->referenceIndices.values[i];
    int currentIndex=context->currentIndices.values[i];
    int noNormal, tooDistant, badCurvature;
    retval = Aligner_processCorrespondence(&error, Htt, Hrr, Htr,
					   bt, br,
					   referenceIndex, 
					   currentIndex, context,
					   &noNormal, &tooDistant, &badCurvature);
  }
  __syncthreads();
  float* dest = sdata + threadIdx.x*56;
  if (retval){
    vecCopy<16>(dest,Htt);
    vecCopy<16>(dest+16,Htr);
    vecCopy<16>(dest+32,Hrr);
    vecCopy<4>(dest+48,bt);
    vecCopy<4>(dest+52,br);
  }
  __syncthreads();
  ReduceMatrix_shared(context->tempsums, threadIdx.x, blockDim.x, blockIdx.x);
};


void Aligner_doCudaStuff(AlignerContext* aligner){
  int numElements = aligner->referenceIndices.rows * aligner->referenceIndices.cols;
  int threadsPerBlock = 256;
  int blocksPerGrid =(numElements + threadsPerBlock - 1) / threadsPerBlock;
  printf("CUDA kernel launch with %d blocks of %d threads\n", blocksPerGrid, threadsPerBlock);
  Aligner_1stKernel<<<blocksPerGrid,threadsPerBlock>>>(aligner->cudaAligner);
  cudaError_t err = cudaGetLastError();
  if (err != cudaSuccess)
    {
      fprintf(stderr, "Failed to launch the 1st kernel\n", cudaGetErrorString(err));
      exit(EXIT_FAILURE);
    }

  while (blocksPerGrid>0){
    int oldBlocksPerGrid=blocksPerGrid;
    blocksPerGrid=blocksPerGrid/threadsPerBlock;
    ReduceMatrix_global<<<blocksPerGrid,threadsPerBlock>>>(aligner->cudaAligner->tempsums, 
							   aligner->cudaAligner->tempsums,
							   oldBlocksPerGrid);

    err = cudaGetLastError();
    if (err != cudaSuccess)
      {
	fprintf(stderr, "Failed to launch the reduction kernel!\n", cudaGetErrorString(err));
	exit(EXIT_FAILURE);
      }
  }
}


#endif

#if 0

void Aligner::align() {
  _projector->setTransform(Isometry3f::Identity());
  _projector->project(_currentIndexImage,
		      _currentDepthImage,
		      *_currentPoints);
  _T = _initialGuess;
  //Vector6f myFuckingT;   myFuckingT << 0.1, 0.01, 0.03, 0, 0, 0;   _T=v2t(myFuckingT);
  // fill the cuda context
  float referenceCurvatures[_referencePoints->size()];
  for (size_t i=0; i<_referencePoints->size(); i++)
    referenceCurvatures[i] = _referenceStats->at(i).curvature();
  
  float currentCurvatures[_currentPoints->size()];
  for (size_t i=0; i<_currentPoints->size(); i++)
    currentCurvatures[i] = _currentStats->at(i).curvature();


  for(int i = 0; i < _outerIterations; i++) {
    cout << "********************* Iteration " << i << " *********************" << endl;
    
    /************************************************************************
     *                         Correspondence Computation                   *
     ************************************************************************/
    cout << "Computing correspondences...";
    
    _projector->setTransform(_T.inverse());
    _projector->project(_referenceIndexImage,
			_referenceDepthImage,
			*_referencePoints);
    
    double tCorrStart = g2o::get_time();
    // Correspondences computation.    
    _correspondenceGenerator.compute(_correspondences,
				     *_referencePoints, *_currentPoints,
				     *_referenceNormals, *_currentNormals,
				     _referenceIndexImage, _currentIndexImage,
				     *_referenceStats, *_currentStats,
				     _T);
    double tCorrEnd = g2o::get_time();
    
    cout << " done." << endl;
    _numCorrespondences = _correspondenceGenerator.numCorrespondences();
    cout << "# inliers found: " << _numCorrespondences << endl;
 
    
    {
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
      
      Aligner_fillContext(&cudaContext,
			  &(_referencePoints->at(0).coeffRef(0,0)), 
			  &(_referenceNormals->at(0).coeffRef(0,0)), 
			  referenceCurvatures,
			  _referencePoints->size(),
			  &(_currentPoints->at(0).coeffRef(0,0)), 
			  &(_currentNormals->at(0).coeffRef(0,0)), 
			  currentCurvatures,
			  &(_currentPointOmegas->at(0).coeffRef(0,0)), 
			  &(_currentNormalOmegas->at(0).coeffRef(0,0)), 
			  _currentPoints->size(),
			  &(_referenceIndexImage.coeffRef(0,0)),
			  &(_currentIndexImage.coeffRef(0,0)),
			  _referenceIndexImage.rows(),
			  _referenceIndexImage.cols(),
			  &_T.matrix().coeffRef(0,0));
      
      cerr << "T: " << _T.matrix() << endl;
      cudaContext.distanceThreshold = 
	_correspondenceGenerator.inlierDistanceThreshold() * 
	_correspondenceGenerator.inlierDistanceThreshold() ;
      cudaContext.flatCurvatureThreshold = _correspondenceGenerator.flatCurvatureThreshold();
      cudaContext.normalThreshold = _correspondenceGenerator.inlierNormalAngularThreshold();
      cudaContext.minCurvatureRatio = 1./_correspondenceGenerator.inlierCurvatureRatioThreshold();
      cudaContext.maxCurvatureRatio = _correspondenceGenerator.inlierCurvatureRatioThreshold();
      cudaContext.inlierThreshold = _linearizer->inlierMaxChi2();
      
    }
    /************************************************************************
     *                            Alignment                                 *
     ************************************************************************/
    for (int k = 0; k < _innerIterations; k++) {      
      Matrix6f H;
      Vector6f b;
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
      double tLinStart = g2o::get_time();
      _linearizer->setT(_T);
      _linearizer->update();
      double tLinEnd = g2o::get_time();
      cerr << "inliers after computing the error" << _linearizer->inliers() << endl;
      //H = _linearizer->H() + Matrix6f::Identity();
      //b = _linearizer->b();

      

      Matrix6f Hgt;
      Vector6f bgt;

      double tStart = g2o::get_time();
      float error;
      float 
	Htt[16], 
	Htr[16], 
	Hrr[16], 
	bt[4], 
	br[4];
      int inliers = Aligner_processCorrespondences(&error, Htt, Htr, Hrr, bt, br, &cudaContext);
      double tEnd = g2o::get_time();
      cerr << "inliers found by the pre-cuda thing:" << inliers << endl;
      

      cerr << "tcorr: " << tCorrEnd - tCorrStart << endl;
      cerr << "tLin: " << tLinEnd - tLinStart << endl;
      cerr << "tMine: " << tEnd - tStart << endl;

      Eigen::Map<Eigen::Matrix4f> _Htt(Htt);
      Eigen::Map<Eigen::Matrix4f> _Htr(Htr);
      Eigen::Map<Eigen::Matrix4f> _Hrr(Hrr);
      Eigen::Map<Eigen::Vector4f> _bt(bt);
      Eigen::Map<Eigen::Vector4f> _br(br);
 
      Hgt.block<3,3>(0,0) = _Htt.block<3,3>(0,0);
      Hgt.block<3,3>(0,3) = _Htr.block<3,3>(0,0);
      Hgt.block<3,3>(3,3) = _Hrr.block<3,3>(0,0);
      Hgt.block<3,3>(3,0) =  Hgt.block<3,3>(0,3).transpose();
      bgt.block<3,1>(0,0) = _bt.block<3,1>(0,0);
      bgt.block<3,1>(3,0) = _br.block<3,1>(0,0);


      cerr << "H difference: " << ((H-Hgt)*1./H.norm()) << endl;
      cerr << "b difference: " << ((b-bgt)*1./b.norm()) << endl;

      H = Hgt;
      b = bgt;

      H+= Matrix6f::Identity() * 10.0f;
      
      Vector6f dx = H.ldlt().solve(-b);
      Eigen::Isometry3f dT = v2t(dx);
      _T = dT * _T;
      _T.matrix().block<1, 4>(3, 0) << 0, 0, 0, 1;
    }    
  }
  _T = _sensorOffset * _T;
}


#endif



/* simple vet test thing */
/*
int main() {
  float v1[4];
  float v2[4];
  for (int i=0; i<4; i++){
    v1[i]=i;
  }
  matPrint(v1,4,1, "v1");
  vecFill(v2, 2, 4);
  matPrint(v2,4,1, "v2");
  vecScale(v1,-1,4);
  matPrint(v1,4,1, "v1=v1*-1");
  printf("v1*v1 = %f\n", vecDot(v1,v1,4));
  vecSum(v1,v2,2.0,4);
  matPrint(v1,4,1, "v1+2*v2");
  vecCopy(v2,v1,4);
  matPrint(v2,4,1, "v2=v1");
  vecScale(v2,0.1,4);
  matPrint(v2,4,1, "v2*.1");
  float m[16];
  for (int i=0; i<4; i++)
    for (int j=0; j<4; j++)
      m[i*4+j]=((i+1)*(j+1))/16.0f;
  matPrint(m,4,4,"m");
  float v3[4];
  matVecMul(v3,m,v2,4,4);
  matPrint(v3,4,1,"m*v2");
  float m3[16];
  matMatMul(m3, m, m, 4,4,4);
  matPrint(m3,4,4,"m*m");
  matBuildSkew(m3,v1);
  matPrint(m3,4,4,"skew(v1)");
  float vm[4];
  matMatMul(vm,m3,v1,4,4,1);
  matPrint(vm,4,1,"non square matrix product");
  
}
*/
