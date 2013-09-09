
inline void toBOSS(boss::ObjectData& data, const std::string name) const {
  boss::ObjectData* matrixData = new boss::ObjectData;
  if (SizeAtCompileTime == Eigen::Dynamic) {
    matrixData->setInt("rows", rows());
    matrixData->setInt("cols", cols());
  }
  boss::ArrayData * adata =  new boss::ArrayData();
  adata->reserve(rows()*cols());
  for (int r=0; r<rows(); r++)
    for (int c=0; c<cols(); c++)
      adata->push_back(new boss::NumberData((float)this->operator()(r,c)));
  matrixData->setField("values",adata);
  data.setField(name, matrixData);
}

inline void fromBOSS(boss::ObjectData& data, const std::string name){
  boss::ObjectData& matrixData = data.getField(name)->getObject();
  if (SizeAtCompileTime == Eigen::Dynamic) {
    int _r = matrixData.getInt("rows");
    int _c = matrixData.getInt("cols");
    if (_r!=0 && _c!=0) 
      this->derived().resize(_r, _c);
    else 
      *this=Matrix<Scalar, RowsAtCompileTime, ColsAtCompileTime>();
  }
  boss::ArrayData& adata = matrixData.getField("values")->getArray();
  assert((int)adata.size()==rows()*cols());
  int k=0;
  for (int r=0; r<rows(); r++)
    for (int c=0; c<cols(); c++, k++)
      this->operator()(r,c) = adata[k];
}


