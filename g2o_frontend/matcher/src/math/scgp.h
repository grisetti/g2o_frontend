#ifndef _AIS_SPGC_HH_
#define _AIS_SPGC_HH_

/** @addtogroup math **/
//@{

namespace AISNavigation{
  /**Implements a spectral clustering workspace.
   Google for "olson, spectral clustering" to get more information.
   Given a similarity matrix of nxn entries, it finds, if any the set
   of entries which are mostly consistent

   When you want to use you should: <br>
   -declare a spectral clusterer<br>
   -resize it to te number og hypotheses and number of classes you want to get
   -compute the dominating eigenvectors with thrpower method
   -discretize the elements of the ith eigenvector to get the elemets of the ith class
*/

  struct SpectralClusterer{
    //@brief Declares a spectral clusterer
    SpectralClusterer();
    ~SpectralClusterer();

    //@brief returns the number of hypotheses
    int dimension() const;

    //@brief returns the number of eigenvalues 
    //(agreeing orthogonal classes)
    int evNum() const;

    //@brief resizes the clusterer
    void resize(int n, int ev_num);
    
    //@brief accessor method to read/write the ith column of the similarity matrix 
    double& a(int col, int row);
    //@brief accessor method to read the nth component of the nth eigenvector
    double& ev(int n, int component);
    //@brief accessor method to read the nth eigenvalue
    double& lambda(int n); 
    //@brief returns true if the component n is accepted after the clustering
    bool mark(int n) const;
    
    //@brief discretizes the ith eigenvalue
    void discretize(int ev_num);

    //@brief calculates dominating eigenvector eigenvalues with the power method
    void powerMethod(int iterations);

  private:
    int _n, _nmax;
    int _ev_num, _ev_max;
    double** _A;
    double** _ev;
    double* _lambda;
    bool* _marks;
  };
} // end namespace

//@}
#endif
