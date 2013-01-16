#include <iostream>
#include <fstream>
#include <stdlib.h>
#include "scgp.h"
#include <stuff/os_specific.h>

using namespace std;
using namespace AISNavigation;


void scgp_test(double factor, int iterations, SpectralClusterer* clusterer){
  
  // fill in a symmetric matrix
  for (int i=0; i<clusterer->dimension(); i++)
    for (int j=i; j<clusterer->dimension(); j++){
      double d=0.3;
      if (drand48()<factor)
	d=1e-20;
      clusterer->a(i,j)=d;
      clusterer->a(j,i)=d;
      if (i==j)
	clusterer->a(i,j)=1e6;
    }

  
  clusterer->powerMethod(iterations);

#ifdef DEBUG
  ofstream os("matrix.dat");
  for (int i=0; i<clusterer->dimension(); i++){
    for (int j=0; j<clusterer->dimension(); j++){
      os << clusterer->a(i,j) << " ";
    }
    os << endl;
  }
  os.close();

  ofstream os2("evec.dat");
  for (int i=0; i<clusterer->evNum(); i++){
    for (int j=0; j<clusterer->dimension(); j++){
      os2 << clusterer->ev(i,j) << " ";
    }
    os2 << endl;
  }
  os2.close();

  ofstream os3("eval.dat");
  for (int i=0; i<clusterer->evNum(); i++){
    os3 << clusterer->lambda(i) << " " << endl;
  }
  os3.close();
#endif

}

int main (int argc, char** argv){
  double factor=0.3;
  int minSize=999;
  int maxSize=1000;

  int eval_num=2;
  int iterations=10;

  SpectralClusterer* clusterer=new SpectralClusterer();

  //int scgp_test(int factor, int size, int eval_num, int iterations, SpectralClusterer* clusterer){
  for (int i=minSize; i<maxSize; i++){
      clusterer->resize(i,eval_num);
      scgp_test(factor, iterations, clusterer);
  }

  ofstream os("matrix.dat");
  for (int i=0; i<clusterer->dimension(); i++){
    for (int j=0; j<clusterer->dimension(); j++){
      os << clusterer->a(i,j) << " ";
    }
    os << endl;
  }
  os.close();

  ofstream os2("evec.dat");
  for (int i=0; i<clusterer->evNum(); i++){
    for (int j=0; j<clusterer->dimension(); j++){
      os2 << clusterer->ev(i,j) << " ";
    }
    os2 << endl;
  }
  os2.close();

  ofstream os3("eval.dat");
  for (int i=0; i<clusterer->evNum(); i++){
    os3 << clusterer->lambda(i) << " " << endl;
  }
  os3.close();
  clusterer->discretize(0);

  delete clusterer;
  return 0;
}
