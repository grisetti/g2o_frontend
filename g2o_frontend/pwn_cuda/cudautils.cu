namespace CudaAligner{

  template <class T>
  __global__ void fillBuffer(T* buffer, int numElems, T value){
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    if (i<numElems){
      buffer[i] = value;
    }
  }


  __global__ void computeDepthBuffer(int* buffer, int rows, int cols, 
				     const float* KT, const float* points, int numPoints,
				     int dmin, int dmax){
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    float ip[4];
    if (i<numPoints){
      const float* wp=points+4*i;
      CudaAligner::matVecMul<4,4>(ip,KT,wp);
      float iw= 1./ip[2];
      int d = ip[2] * 1000.0f;
      int x = (int)(ip[0]*iw);
      int y = (int)(ip[1]*iw);
      int offset = y*rows+x;
      if (d > dmin && d < dmax &&
	  x >= 0 && x<rows && 
	  y>=0 && y<cols){
	atomicMin(buffer+offset,d);
      }
    }
  }


  __global__ void computeIndexBuffer(int* ibuffer, const int* zbuffer, int rows, int cols, 
				     const float* KT, const float* points, int numPoints){
    int i = blockDim.x * blockIdx.x + threadIdx.x;
    float ip[4];
    if (i<numPoints){
      const float* wp=points+4*i;
      CudaAligner::matVecMul<4,4>(ip,KT,wp);
      float iw= 1./ip[2];
      int d = ip[2] * 1000.0f;
      int x = (int)(ip[0]*iw);
      int y = (int)(ip[1]*iw);
      int offset = y*rows+x;
      if ( x >= 0 && x<rows && 
	   y>=0 && y<cols && d == zbuffer[offset]){
	ibuffer[offset]=i;
      }
    }
  }


  __device__ int processCorrespondence(float* Htt,
				       float* Hrr,
				       float* Htr,
				       float* bt,
				       float* br,
				       const float* transform,
				       const float* referencePoints,
				       const float* referenceNormals,
				       const float* referenceCurvatures,
				       const float* currentPoints,
				       const float* currentNormals,
				       const float* currentCurvatures,
				       const float* currentOmegaPs,
				       const float* currentOmegaNs,
				       const int referenceIndex, 
				       const int currentIndex,
				       const float flatCurvatureThreshold,
				       const float normalThreshold,
				       const float minCurvatureRatio,
				       const float maxCurvatureRatio,
				       const float distanceThreshold, 
				       const float inlierThreshold){

    vecFill<16>(Htt, 0);
    vecFill<16>(Htr, 0);
    vecFill<16>(Hrr, 0);
    vecFill<4>(bt,0);
    vecFill<4>(br,0);
    const float* T=transform;
    float referencePoint[4];
    {
      const float* referencePoint_  =  referencePoints + (referenceIndex*4);
      matVecMul<4,4>(referencePoint,T,referencePoint_);
    }

    float referenceNormal[4];
    {
      const float* referenceNormal_ =  referenceNormals + (referenceIndex*4);
      matVecMul<4,4>(referenceNormal,T,referenceNormal_);
    }

    float referenceCurvature      =  referenceCurvatures[referenceIndex];
    referenceCurvature = (referenceCurvature<flatCurvatureThreshold)?flatCurvatureThreshold:referenceCurvature;


    const float* currentPoint     =  currentPoints + ( currentIndex* 4);
    const float* currentNormal    =  currentNormals + ( currentIndex* 4);
    float   currentCurvature      =  currentCurvatures[currentIndex];
    currentCurvature = (currentCurvature<flatCurvatureThreshold)?flatCurvatureThreshold:currentCurvature; 

    const float* omegaP           =  currentOmegaPs + ( currentIndex* 16);
    const float* omegaN           =  currentOmegaNs + ( currentIndex* 16);
    float pointsDifference[4];



    float curvatureRatio=(referenceCurvature + 1e-5)/(currentCurvature + 1e-5);
    float normalsRatio = vecDot<4>(currentNormal,referenceNormal);
  
    vecCopy<4>(pointsDifference,referencePoint);
    vecSum<4>(pointsDifference,currentPoint,-1.0f);
    float pointsDistance=vecDot<4>(pointsDifference,pointsDifference);


    bool normalFail = (normalsRatio < normalThreshold);
    bool distanceFail = (pointsDistance > distanceThreshold);
    bool curvatureFail = ((curvatureRatio < minCurvatureRatio) ||
			  (curvatureRatio > maxCurvatureRatio));

    int increment = ! normalFail && ! distanceFail && ! curvatureFail;
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
  
    float localError = vecDot<4>(ep,pointsDifference) + vecDot<4>(en,normalsDifference);

    int chiOk = localError < inlierThreshold;
    if (! chiOk)
      return 0;
  
    //Matrix4f Sp = skew(referencePoint);
    float Sp[16];
    matBuildSkew(Sp,referencePoint);

    //Matrix4f Sn = skew(referenceNormal);
    float Sn[16];
    matBuildSkew(Sn,referenceNormal);


    //Htt = omegaP;
    vecCopy<16>(Htt,omegaP);
  
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
    *(bt+3)=1;
    *(br+3)= localError;
    return 1;
  }

}
