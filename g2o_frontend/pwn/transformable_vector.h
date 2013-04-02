#ifndef _TRANSFORMABLE_VECTOR_4F_H_
#define _TRANSFORMABLE_VECTOR_4F_H_
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <iostream>

template <typename Transformable>
struct TransformableVector: public std::vector<Transformable, Eigen::aligned_allocator<Transformable> > {
  TransformableVector() : std::vector<Transformable, Eigen::aligned_allocator<Transformable> >() {
  }
  TransformableVector(size_t s) : std::vector<Transformable, Eigen::aligned_allocator<Transformable> >(s){
  }
  
  template<typename OtherDerived>
  inline void transformInPlace(const OtherDerived& m){
    Transformable* t= &(*this)[0];
    for (size_t i=0; i< std::vector<Transformable, Eigen::aligned_allocator<Transformable> >::size(); i++, t++){
      *t=m*(*t);
    }
  }

  template<typename OtherDerived>
  inline void transform(TransformableVector& dest, const OtherDerived& m) const{
    dest.resize(this->size());
    const Transformable* tSrc= &(*this)[0];
    Transformable* tDest= &dest[0];
    for (size_t i=0; i< std::vector<Transformable, Eigen::aligned_allocator<Transformable> >::size(); ++i, ++tSrc, ++tDest ){
      *tDest=m*(*tSrc);
    }
  }

};

#endif
