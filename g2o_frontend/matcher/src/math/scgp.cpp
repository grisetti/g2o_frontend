#include "scgp.h"
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <stuff/os_specific.h>

#define MAXSIZE 1000

namespace AISNavigation{

  static double _vvProd(double* v1, double*v2, int n){
    double a=0;
    for(int i=0; i<n; i++){
      a+=(*v1)*(*v2);
      v1++;
      v2++;
    }
    return a;
  }
  
  static void _vCopy(double* to, double* from, int n){
    memcpy(to, from, n*sizeof(double));
  }

  static void _vmProd (double* result, double** A, double* b, int r, int c){
    for (int i=0; i<r; i++){
      double a=0.;
      double* aptr=A[i];
      double* bptr=b;
      for (int j=0; j<c; j++) {
	a+=(*aptr)*(*bptr);
	aptr++;
	bptr++;
      }
      *result=a;
      result++;
    }
  }
  
  static void _vScale(double* v, double k, int n){
    for(int i=0; i<n; i++){
      *v*=k;
      v++;
    }
  }

  static void _vvsSum(double* result, double*v2, double k, int n){
    for (int i=0; i<n; i++){
      *result += (*v2)*k;
      result++;
      v2++;
    }
  } 
  

  static void _powerMethod (double* lambda, double** ev, double** A, int n, int ev_num, int iterations){
    assert(ev_num<=n);
    assert(iterations>0);
    assert (n<MAXSIZE);
    double tmp[MAXSIZE];
    for (int i=0; i<ev_num; i++){
      for (int q=0; q<n; q++){
	ev[i][q]=drand48();
      }
      double a=_vvProd(ev[i],ev[i],n);
      a=1./sqrt(a);
      _vScale(ev[i],a,n);
      
      double* evi=ev[i];
      for (int it=0; it<iterations; it++){
	// subtract the components of the 
	for (int j=0; j<i; j++){
	  double* evj=ev[j];
	  a=_vvProd(evi,evj,n);
	  _vvsSum(evi,evj,-a,n);
	}
	double inorm=1./sqrt(_vvProd(evi,evi,n));
	_vScale(evi,inorm,n);
	_vmProd(tmp,A,evi,n,n);
	_vCopy(evi,tmp,n);
	double norm=sqrt(_vvProd(evi,evi,n));
	lambda[i]=norm;
      }
      double inorm=1./sqrt(_vvProd(evi,evi,n));
      _vScale(evi,inorm,n);
    }
  }

  
  SpectralClusterer::SpectralClusterer(){
    _n=0;
    _nmax=0;
    _ev_num=0;
    _ev_max=0;
    _A=0;
    _ev=0;
    _lambda=0;
    _marks=0;
  }

  int SpectralClusterer::dimension() const{
    return _n;
  }
  
  int SpectralClusterer::evNum() const{
    return _ev_num;
  }

  SpectralClusterer::~SpectralClusterer(){
    if (_nmax==0)
      return;

    for (int i=0; i<_nmax; i++)
      delete [] _A[i];
    delete [] _A;
    delete [] _marks;

    for (int i=0; i<_ev_max; i++){
      delete [] _ev[i];
    }
    delete [] _ev;
    delete [] _lambda;
  }
  
  void SpectralClusterer::resize(int n, int ev_num){
    assert (n>0);
    assert (ev_num>0);
    int nmax=_nmax>n?_nmax:n;
    int evmax=_ev_max>ev_num?_ev_max:ev_num;

    if (n>_nmax){
      if (_A){
	for (int i=0; i<_nmax; i++)
	  delete [] _A[i];
	delete [] _A;
	delete [] _marks;
      }

      _A=new double* [nmax];
      for (int i=0; i<nmax; i++)
	_A[i]=new double[nmax];
      _marks= new bool[nmax];
    }
    
    if (ev_num>_ev_max || n>_nmax){
      if (_ev){
	for (int i=0; i<_ev_max; i++)
	  delete [] _ev[i];
	delete [] _ev;
      }

      _ev=new double*[evmax];
      for (int i=0; i<evmax; i++){
	_ev[i]=new double[nmax];
      }
      
    }
    
    if (ev_num>_ev_max){
      if (_lambda)
	delete [] _lambda;
      _lambda=new double[evmax];
    }
 
    _nmax=nmax;
    _ev_max=evmax;
    _n=n;
    _ev_num=ev_num;
  }
  
  double& SpectralClusterer::a(int col, int row){
    assert (col>=0);
    assert (row>=0);
    assert (col<_n);
    assert (row<_n);
    return _A[row][col];
  }

  bool SpectralClusterer::mark(int n) const{
    assert (n>=0);
    assert(n<_n);
    return _marks[n];
  }
  
  double& SpectralClusterer::ev(int n, int component){
    assert (n>=0);
    assert (component>=0);
    assert (n<_ev_num);
    assert (component<_n);
    return _ev[n][component];
  }
  
  double& SpectralClusterer::lambda(int n){
    assert (n>=0);
    assert (n<_ev_num);
    return _lambda[n];
  }

  void SpectralClusterer::powerMethod(int iterations){
    assert (_n);
    assert (_ev_num);
    _powerMethod (_lambda, _ev, _A, _n, _ev_num,  iterations);
  }

  struct _DPVIComp{
    double* v;
    bool operator()(int i1, int i2){
      return v[i1]>v[i2];
    }
  };

  void SpectralClusterer::discretize(int ev_num){
    using namespace std;
    assert(_n);
    assert(_n<MAXSIZE);
    assert(ev_num<_ev_num);
    

    _DPVIComp compare;
    compare.v=_ev[ev_num];
    int indices[MAXSIZE];

    double *v=_ev[ev_num];;
    for (int i=0; i<dimension(); i++)
      indices[i]=i;
    std::sort(indices, indices+dimension(), compare);

    double previousSqNorm=1;
    double previousNorm=1;
    double previousProduct=v[indices[0]];
    double previousValue=v[indices[0]];
    for (int i=0; i<dimension(); i++){
      int k=indices[i];
      double newProduct=previousProduct+v[k];
      double newSqNorm=previousSqNorm+1;
      double newNorm=sqrt(newSqNorm);
      double newValue=newProduct/newNorm;
//        cerr.setf(ios::fixed);
//        cerr << setprecision(6);
//        cerr <<  i << "\t" << k << "\t" << v[k] << "\t" <<  newProduct << "\t"  << previousProduct << newNorm << "\t"<< "\t" << previousNorm << "\t";
      if (newValue>previousValue){
	previousNorm=newNorm;
	previousSqNorm=newSqNorm;
	previousProduct=newProduct;
	previousValue=newValue;
	_marks[k]=true;
      } else 
	_marks[k]=false;
      //      cerr << _marks[k] << endl;
    }
  }
  
};
