#include <algorithm>
#include <iostream>
#include <limits>
#include "vector_n.h"
#include "matrix_n.h"
#include "rotation_matrix.h"
#include "quaternion.h"
#include "axis_angle.h"
#include "angle.h"
#include "transformation.h"

#include <stdlib.h>
#include <stuff/macros.h>
#include <vector>

using namespace std; 


// template <typename V>
// struct LexCompare{
//   LexCompare(int dmax=std::numeric_limits<int>::max()){
//     _dmax=dmax;
//   }
//   inline bool operator()(const V& v1, const V& v2 ) const {
//     const int s=std::min(_dmax, v1.size());
//     for (int i=0; i<s; i++){
//       if (v1[i]<v2[i])
// 	return true;
//       else if (v1[i]<v2[i])
// 	return false;
//     }
//     return false;
//   }
//   int _dmax;
// };


// template <typename I>
// void cluster(std::vector<_Vector<2*V::_N, typename V::BaseType> >& dest, const std::vector< V >& src, const I::value_type& resolution){
//   // compute the inverse of the resolution
//   const int N=V::_N;
//   V mag;
//   for (int i=0; i<N; i++)
//     mag[i]=1./resolution[i];
//   dest.resize(src.size);
//   // construct an enlarged vector, the first part is the indices, the second part is the original point
//  for (int i=0; i<src.size(); i++){
//    const V& sv=src[i];
//    _Vector<2*N, typename V::BaseType>& dv=dest[i];
//    for (int k=0; k<N; k++){
//      dv[k]=(int) (sv[k]*mag[k]);
//      dv[k+N]=sv[k];
//    }
//  }
//  // sort it by considering only the indices
//  LexCompare< _Vector<2*N, typename V::BaseType> > lc(N);
//  std::sort(dest.begin(), dest.end(), lc);
// }

// template <typename V>
// struct VectorStats{
//   typename V::BaseType BaseType;
//   typename V           VectorType;
//   const int _N=V::_N;
//   typename _Matrix<BaseType, _N, _N> CovarianceType;
//   VectorStats();
//   const VectorStats();
//   void add(const V& v);
//   void remove(const V& v);
//   const V& normal() const;
//   int  _size;
//   mutable V _normal;
//   V _normalAcc;
//   V _eigenvalues;
//   mutable bool _normalComputed;
// }

int main(int argc, char** argv){
  (void)argc; (void)argv;  // No warnings

  cerr <<  "Vector Test" << endl;
  cerr << "construction" << endl;
  Vector3 v1(1.,2.,3.);
  cerr << PVAR(v1) << endl;
  cerr << "copy constructor and scalar product -1" << endl;
  Vector3 v2(v1*(-1.));
  cerr << PVAR(v2);
  cerr << "assignment and sum" << endl;
  Vector3 v3=v1+v2;
  cerr << PVAR(v1);
  cerr << PVAR(v3);
  cerr << "scalarProduct v1*v2 " << endl;
  cerr << PVAR(v1);
  cerr << PVAR(v2);
  cerr << v1*v2 << endl;
  cerr << "norm(v1)" << endl;
  cerr << v1.norm() << endl;
  cerr << "v1+=(v2*3.)" << endl;
  cerr << (v1+=(v2*3.));
  cerr << "v1-=(v2*3.)" << endl;
  cerr << (v1-=(v2*3.));

  cerr <<  "Matrix Test" << endl;

  Matrix6 m6;
  for (int j=0; j<m6.rows(); j++){
    for (int i=0; i<m6.cols(); i++){
      m6[j][i]=pow((double)i+1,(double)j);
    }
  }
  cerr << PVAR(m6);
  cerr << PVAR(m6.det());
  cerr << PVAR(m6.inverse());
  cerr << PVAR(m6.inverse()*m6);
  typedef _Matrix<5, 6> Matrix5x6d;
  Matrix5x6d m5x6, m5x6a;
  for (int j=0; j<m5x6.rows(); j++){
    for (int i=0; i<m5x6.cols(); i++){
      m5x6[j][i]=(i==j)?(1.):(-1.);
    }
  }
  cerr << PVAR(m5x6);
  cerr << "m5x6*=0.5" << endl;
  m5x6*=0.5;
  cerr << PVAR(m5x6);

  _Matrix<0,0,double> dm5x6;
  st2dyn(dm5x6,m5x6);
  cerr << PVAR(dm5x6);

  dm5x6=dm5x6*dm5x6.transpose();
  cerr << "ROWS,COLS=" << dm5x6.rows() << "," << dm5x6.cols() << endl;
  cerr << dm5x6 << endl;

  _Matrix<5,5,double> xxx;
  dyn2st(xxx,dm5x6);
  cerr << xxx << endl;


 

  cerr << "svd" << endl;
  _Matrix<5, 6> u;
  _Vector<6> s;
  _Matrix<6, 6, double> v;
  m5x6.svd(u,s,v);
  
  cerr << "difference" << endl;
  cerr << (m5x6-(u*_Matrix<6,6,double>::diag(s)*v.transpose())) << endl;

  cerr << PVAR(s) << endl;
  cerr << PVAR(u) << endl;
  cerr << PVAR(v) << endl;

  cerr << (m5x6a=m5x6*m6);
  
  typedef _Matrix<5, 5, double> Matrix5d;
  Matrix5d m5=m5x6a*m5x6a.transpose();
  cerr << PVAR(m5);
  

  Matrix5d m5c=m5.cholesky();
  Matrix5d m5ci = m5c.inverse();
  Matrix5d m5i = m5ci.transpose()*m5ci;
  
  cerr <<  endl << "Chol inverse" << endl;
  cerr << PVAR(m5i) << endl;

  cerr <<  endl << "Test" << endl;
  cerr << PVAR(m5i*m5) << endl;

  cerr << PVAR(m5*m5.inverse()) << endl;



  return 0;



  cerr <<  "cholesky" << endl;
  cerr << PVAR(m5.cholesky());
  m5=m5.cholesky();
  cerr << "chol*chol";
  cerr << m5*m5.transpose() << endl;
  cerr <<  (m5*(double)2.);


  

  cerr << "rotations" << endl;


  RotationMatrix3 r(M_PI/3, M_PI/3, M_PI/3);
  Quaternion q(r);
  AxisAngle a(r);
  
  cerr << PVAR(r) << endl;
  cerr << PVAR(q) << endl;
  cerr << PVAR(a) << endl;

  cerr << PVAR(r.angles());
  cerr << PVAR(q.angles());
  cerr << PVAR(a.angles());

  r=RotationMatrix3(0.,0.,M_PI/10.);
  q=Quaternion(r);
  a=AxisAngle(r);

  cerr << PVAR(r) << endl;
  cerr << PVAR(q) << endl;
  cerr << PVAR(a) << endl;

  cerr << PVAR(r.angles());
  cerr << PVAR(q.angles());
  cerr << PVAR(a.angles());

  cerr << "self composition" << endl;
  r*=r;
  q*=q;
  a*=a;

  cerr << PVAR(r) << endl;
  cerr << PVAR(q) << endl;
  cerr << PVAR(a) << endl;

  cerr << PVAR(r.angles());
  cerr << PVAR(q.angles());
  cerr << PVAR(a.angles());



  cerr << "multiply by inverse" << endl;
  r*=r.inverse();
  q*=q.inverse();
  a*=a.inverse();
  
  cerr << PVAR(r) << endl;
  cerr << PVAR(q) << endl;
  cerr << PVAR(a) << endl;

  cerr << PVAR(r.angles());
  cerr << PVAR(q.angles());
  cerr << PVAR(a.angles());

  r=RotationMatrix3(0.,0.,M_PI/10.);
  q=Quaternion(0.,0.,M_PI/10.);
  a=AxisAngle(0.,0.,M_PI/10.);

  cerr << PVAR(r.angles()) << endl;
  cerr << PVAR(q.angles()) << endl;
  cerr << PVAR(a.angles()) << endl;
  

  RotationMatrix2 r2(50);
  Angle a2(r2.angle());
  cerr << PVAR(r2.angle());
  cerr << PVAR(a2.angle());

  v3=Vector3 (10,2.,34);
  Transformation3 t3q(v3, q);
  cerr << PVAR(t3q.translation());
  cerr << PVAR(t3q.rotation());
  Transformation3 t3q_a(t3q);
  t3q=t3q*t3q_a.inverse();
  cerr << PVAR(t3q.translation());
  cerr << PVAR(t3q.rotation());


  cerr << PVAR(dm5x6) << endl;
  _Matrix<0,0,double> dm5x6_2;
  dm5x6_2 = dm5x6;
  cerr << PVAR(dm5x6_2) << endl;


  _Matrix <8,9,double> ntest;
  for (int i=0; i<ntest.rows(); i++)
    for (int j=0; j<ntest.cols(); j++)
      ntest[i][j]=1e3*drand48();

  _Matrix <8,9,double> nullSpace;

  cerr << PVAR(ntest) << endl;
  cerr << "nsSize=" << ntest.nullSpace(nullSpace, 1e-6) << endl;
  cerr << nullSpace << endl;

  cerr << PVAR(ntest*nullSpace.transpose()) << endl;

}

