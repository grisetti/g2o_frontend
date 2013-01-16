#include <iostream>
#include "array_allocator.h"
#include "timeutil.h"

using namespace std;

typedef _ArrayAllocator <0, double> DynDouble_Array;

int main (int argc, char** argv) {
  double t = get_monotonic_time();
  double tw = get_time();
  for (int i = 0; i < 10; ++i) {
    cerr << "monotonic " << get_monotonic_time() - t;
    cerr << "\twall " << get_time() - tw << endl;
    usleep(10000);
  }
  return 0;


  DynDouble_Array da1(10);
  for (int i=0; i<da1.size(); i++)
    da1.ptr()[i]=i;

  DynDouble_Array da2(0);
  da2=da1;

  for (int i=0; i<da2.size(); i++)
    cerr << da2.ptr()[i] << " ";
  cerr << endl;

  _ArrayAllocator<5,double> sa1;
  for (int i=0; i<sa1.size(); i++)
    sa1.ptr()[i] =  i;
  for (int i=0; i<sa1.size(); i++)
    cerr << sa1.ptr()[i] << " ";
  cerr << endl;
  _ArrayAllocator<5,double> sa2=sa1;
  for (int i=0; i<sa2.size(); i++)
    cerr << sa2.ptr()[i] << " ";
  cerr << endl;

  _ArrayAllocator<10,double> sa3;
  dyn2st(sa3,da1);

  for (int i=0; i<sa3.size(); i++)
    cerr << sa3.ptr()[i] << " ";
  cerr << endl;
  st2dyn(da1,sa1);


  for (int i=0; i<sa3.size(); i++)
    cerr << sa3.ptr()[i] << " ";
  cerr << endl;

  _Array2DAllocator<3,9,double> m39;


}

