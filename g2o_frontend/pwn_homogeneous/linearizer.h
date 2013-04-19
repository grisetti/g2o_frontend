class Linearizer() {
 public:
  Linearizer() {};

  void computeAnalyticJacobian();
  void computeNumericalJacobian();

  void computeH();
  void computeB();

 protected:

  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> H;
  Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> b;
};
