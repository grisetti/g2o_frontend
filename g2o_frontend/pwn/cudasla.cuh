#ifndef _CUDASLA_H_
#define _CUDASLA_H_

__host__ __device__ void vecFill(float* v, float x, int n){
  for(int i=0; i<n; i++)
    v[i]=x;
}

__host__ __device__ void vecScale(float* v, float s, int n){
  for(int i=0; i<n; i++)
    v[i]*=s;
}

__host__ __device__ void vecCopy(float* dest, const float* src, int n){
  for(int i=0; i<n; i++)
    dest[i]=src[i];
}

__host__ __device__ void vecSum(float*dest, const float* src, float scale, int n){
  for(int i=0; i<n; i++)
    dest[i]+=scale*src[i];
}

__host__ __device__ float vecDot(const float* v1, float* v2, int n){
  float a=0;
  for(int i=0; i<n; i++)
    a+=v1[i]*v2[i];
  return a;
}

__host__ __device__ void matVecMul(float* dest, const float* A, const float*b, int rows, int cols){
  vecFill(dest, 0, rows);
  for (int i=0; i<cols; i++){
    vecSum(dest,A,b[i],rows);
    A+=rows;
  }
}

__host__ __device__ void matMatMul(float* dest, const float* A, const float*B, int ra, int ca, int cb){
  float* dptr = dest;
  const float* bptr = B;
  for (int i=0; i<cb; i++){
    matVecMul(dptr,A,bptr,ra,ca);
    dptr += ra;
    bptr += ca;
  }
}


__host__ __device__ void matTranspose(float* dest, const float* src, int rows, int cols){
  for (int i=0; i<cols; i++)
    for (int j=0; j<rows; j++)
      dest[j*cols+i] = src[i*rows+j];
}


template <int n>
__host__ __device__ void vecFill(float* v, float x){
  for(int i=0; i<n; i++)
    v[i]=x;
}

template <int n>
__host__ __device__ void vecScale(float* v, float s){
  for(int i=0; i<n; i++)
    v[i]*=s;
}

template <int n>
__host__ __device__ void vecCopy(float* dest, const float* src){
  for(int i=0; i<n; i++)
    dest[i]=src[i];
}

template <int n>
__host__ __device__ void vecSum(float*dest, const float* src, float scale){
  for(int i=0; i<n; i++)
    dest[i]+=scale*src[i];
}

template <int n>
__host__ __device__ float vecDot(const float* v1, const float* v2){
  float a=0;
  for(int i=0; i<n; i++)
    a+=v1[i]*v2[i];
  return a;
}

template <int rows, int cols>
__host__ __device__ void matVecMul(float* dest, const float* A, const float*b){
  vecFill<rows>(dest, 0);
  for (int i=0; i<cols; i++){
    vecSum<rows>(dest,A,b[i]);
    A+=rows;
  }
}

template <int ra, int ca, int cb>
__host__ __device__ void matMatMul(float* dest, const float* A, const float*B){
  float* dptr = dest;
  const float* bptr = B;
  for (int i=0; i<cb; i++){
    matVecMul<ra,ca>(dptr,A,bptr);
    dptr += ra;
    bptr += ca;
  }
}


template <int rows, int cols>
__host__ __device__ void matTranspose(float* dest, const float* src){
  for (int i=0; i<cols; i++)
    for (int j=0; j<rows; j++)
      dest[j*cols+i] = src[i*rows+j];
}

__host__ __device__ void matBuildSkew(float* m, const float* v){
  const float x = 2*v[0];
  const float y = 2*v[1];
  const float z = 2*v[2];
  m[0] =  0;   m[4] =  z;  m[8]  = -y; m[12] = 0;   
  m[1] = -z;   m[5] =  0;  m[9]  =  x; m[13] = 0;   
  m[2] =  y;   m[6] = -x;  m[10] =  0; m[14] = 0;   
  m[3] =  0;   m[7] =  0;  m[11] =  0; m[15] = 0;   
}

__host__ __device__ void transformInverse(float* d, const float* s) {
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

__host__ __device__ void _v2t(float* m, const float* v) {
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

__host__ __device__ void _t2v(float* v, const float* m) {
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

#endif
