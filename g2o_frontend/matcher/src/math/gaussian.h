#ifndef _GAUSSIAN_H_
#define _GAUSSIAN_H_
#include "vector_n,h"
#include "matrix_n,h"

template <int N, typename Base>
  struct _Gaussian {
    _Gaussian(int s);
    _Gaussian(const Vector<N, Base>& v_, const Matrix<N,N,Base> m_, bool isCovForm);
    const Vector<N, Base> mean() const;
    const Matrix<N, N, Base> covariance() const;
    const Vector<N, Base> infoVector() const;
    const Matrix<N, N, Base> information() const;
    
    void setMean(const Vector<N, Base>& mean_);
    void setCovariance(const Matrix<N, N, Base>& cov_);
    void setInformationVector(const Vector<N, Base>& iv_);
    void setInformationMatrix(const Matrix<N, N, Base>& cov_);

    _Gaussian<N, Base> operator + (const _Gaussian <N, Base>& g);
    
  protected:
    updateInfoForm();
    updateCovform();
    mutable _Matrix<N,N,Base> _covariance;
    mutable _Matrix<N,N,Base> _information;
    mutable _Vector<N,Base> _mean;
    mutable _Vector<N,Base> _informationVector;
    mutable bool _infoFormUpdated;
    mutable bool _covarianceFormUpdated;
  };

template <int N, int M, typename Base>
  _Gaussian<N,Base> operator * (_Matrix<M,N,Base> m, _Gaussian<M,Base> g){
}


#endif
