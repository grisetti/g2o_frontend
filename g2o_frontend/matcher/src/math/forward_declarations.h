#ifndef AISLIB_MATH_FORWARD_DECLARATIONS_H
#define AISLIB_MATH_FORWARD_DECLARATIONS_H

/** This header can be included to use forward declarations of the math classes */


// Vector
template <int N, typename Base>
struct _Vector;
typedef _Vector<0,double> VectorX;
typedef _Vector<0,float>  VectorXf;
typedef _Vector<2,double> Vector2;
typedef _Vector<2,int>    Vector2i;
typedef _Vector<2,float>  Vector2f;
typedef _Vector<3,double> Vector3;
typedef _Vector<3,float>  Vector3f;
typedef _Vector<6,double> Vector6;
typedef _Vector<6,float>  Vector6f;


// Matrix
template <int Rows, int Cols, typename Base>
struct _Matrix;
typedef _Matrix<0, 0, double> MatrixX;
typedef _Matrix<0, 0, float>  MatrixXf;
typedef _Matrix<2, 2, double> Matrix2;
typedef _Matrix<2, 2, float>  Matrix2f;
typedef _Matrix<3, 3, double> Matrix3;
typedef _Matrix<3, 3, float>  Matrix3f;
typedef _Matrix<6, 6, double> Matrix6;
typedef _Matrix<6, 6, float>  Matrix6f;


#endif
