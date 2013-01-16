#ifndef AISSTAT_HH
#define AISSTAT_HH

namespace AISNavigation {
/** @addtogroup math **/
//@{

//@brief returns a sample extracted from a triangular distribution.
// Width is the base of the triangle
double triangularSample(double width, double mean=0);


//@}
};

#endif
