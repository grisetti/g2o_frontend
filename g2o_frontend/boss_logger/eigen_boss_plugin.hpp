
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
      adata->push_back(new boss::NumberData(this->operator()(r,c)));
  matrixData->setField("values",adata);
  data.setField(name, matrixData);
}

inline void fromBOSS(boss::ObjectData& data, const std::string name){
  boss::ObjectData* matrixData = static_cast<boss::ObjectData*>(data.getField(name));
  if (SizeAtCompileTime == Eigen::Dynamic) {
    int _r = matrixData->getInt("rows");
    int _c = matrixData->getInt("cols");
    this->resize(_r, _c);
  }
  boss::ArrayData* adata = static_cast<boss::ArrayData*>(matrixData->getField(name));
  assert(adata->size()==rows()*cols());
  int k=0;
  for (int r=0; r<rows(); r++)
    for (int c=0; c<cols(); c++, k++)
      this->operator()(r,c) = adata[k];
}


