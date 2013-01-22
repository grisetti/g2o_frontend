#include "scanMatcher.h"

using namespace std;
using namespace Eigen;



float ScanMatcherResult::matchingScore() const
{
  return _matchingScore;
}


ScanMatcherResult::~ScanMatcherResult() {}



ScanMatcher::ScanMatcher(const float& resolution, const float& radius, const int& kernelSize, const float& kernelMaxValue)
{
  int size = (int) 2*(radius/resolution);
  this->_scanGrid = FloatGrid(Vector2i(size, size), Vector2f(-radius, -radius), resolution, float(0));
  this->_convolvedGrid = FloatGrid(Vector2i(size, size), Vector2f(-radius, -radius), resolution, float(0.5));
  this->initializeKernel(kernelSize, resolution, kernelMaxValue);
  
//   _rasterCells.reserve(size*size);
//   _convolvedCells.reserve(size*size);
//   _rasterIndices.reserve(size*size);
//   _convolvedIndices.reserve(size*size);
}


ScanMatcher::ScanMatcher(const _GridMap<float>& inputGrid, const int& kernelSize, const float& kernelMaxValue)
{
  this->_scanGrid = inputGrid;
  int size = 2*(_scanGrid.lowerLeft().x()/_scanGrid.resolution());

//   _rasterCells.reserve(size*size);
//   _convolvedCells.reserve(size*size);
//   _rasterIndices.reserve(size*size);
//   _convolvedIndices.reserve(size*size);
  
  for(int x = 0; x < inputGrid.size().x(); ++x)
  {
    for(int y = 0; y < inputGrid.size().y(); ++y)
    {
      Vector2i currentIndices(x,y);
      if(inputGrid.cell(currentIndices) > 0)
      {
	_rasterIndices.push_back(currentIndices);
	_rasterCells.push_back(&_scanGrid.cell(currentIndices));
      }
    }
  }
  
  this->_convolvedGrid = FloatGrid(inputGrid.size(), inputGrid.lowerLeft(), inputGrid.resolution(), float(0.5));
  this->initializeKernel(kernelSize, inputGrid.resolution(), kernelMaxValue);
  this->convolveGrid(_scanGrid);
}


ScanMatcher::~ScanMatcher()
{
  clear();
}


void ScanMatcher::clear()
{
  int sizeCC = _convolvedCells.size();
  for(int i = 0; i < sizeCC; ++i)
  {
    *(_convolvedCells[i]) = 0.5;
  }
  _convolvedCells.clear();
  _convolvedIndices.clear();  
  
  int sizeRC = _rasterCells.size();
  for(int j = 0; j < sizeRC; ++j)
  {
    *(_rasterCells[j]) = 0;
  }
  _rasterCells.clear();
  _rasterIndices.clear();
}


void ScanMatcher::convolveGrid(const _GridMap<float>& g)
{
  const float* ker = _kernel.data();
  int kRows = _kernel.rows();
  int kCols = _kernel.cols();
  Vector2i gridSize = _convolvedGrid.size();
  int oRows = gridSize.x();
  int oCols = gridSize.y();
  int center = (kRows-1)/2;
  
  vector<Vector2i> gRasterIndices = this->getRasterIndices();
  for(vector<Vector2i>::const_iterator it = gRasterIndices.begin(); it != gRasterIndices.end(); ++it)
  {
    Vector2i ip = *it;
    int r = ip.x();
    int c = ip.y();
    for(int i = 0; i < kRows; ++i)
    {
      int iOut = r+i-center;
      if((iOut >= 0) && (iOut < oRows))
      {
	for(int j = 0; j < kCols; ++j)
	{
	  int jOut = c+j-center;
	  if((jOut >= 0) && (jOut < oCols))
	  {
	    float& v = _convolvedGrid.cell(iOut, jOut);
	    const float& k = ker[j*kRows+i];
	    v = (k<v) ? k : v;
	    Vector2i currentIndices(iOut, jOut);
	    _convolvedIndices.push_back(currentIndices);
	    _convolvedCells.push_back(&_convolvedGrid.cell(currentIndices));
	  }
	}
      }
    }
  }
}


void ScanMatcher::convolveScan(const ScanMatcher::Vector2fVector& ns, const Isometry2f& transform)
{
  const float* ker = _kernel.data();
  int kRows = _kernel.rows();
  int kCols = _kernel.cols();
  Vector2i gridSize = _convolvedGrid.size();
  int oRows = gridSize.x();
  int oCols = gridSize.y();
  int center = (kRows-1)/2;

  Matrix2f matrix = transform.rotation();
  float angle = atan2(matrix(1, 0), matrix(0, 0));
  Vector2f offset = transform.translation();
  float c = cos(angle);
  float s = sin(angle);
  float tx = offset.x();
  float ty = offset.y();
  
  for(Vector2fVector::const_iterator it = ns.begin(); it != ns.end(); ++it)
  {
    Vector2f p = *it;
    float px = p.x();
    float py = p.y();
    Vector2f transformedPoint(c*px - s*py + tx, s*px + c*py + ty);
    Vector2i ip = this->_scanGrid.world2grid(transformedPoint);    
    int r = ip.x();
    int c = ip.y();
    for(int i = 0; i < kRows; ++i)
    {
      int iOut = r+i-center;
      if((iOut >= 0) && (iOut < oRows))
      {
	for(int j = 0; j < kCols; ++j)
	{
	  int jOut = c+j-center;
	  if((jOut >= 0) && (jOut < oCols))
	  {
	    float& v = _convolvedGrid.cell(iOut, jOut);
	    const float& k = ker[j*kRows+i];
	    v = (k<v) ? k : v;
	    Vector2i currentIndices(iOut, jOut);
	    _convolvedIndices.push_back(currentIndices);
	    _convolvedCells.push_back(&_convolvedGrid.cell(currentIndices));
	  }
	}
      }
    }
  }
}


void ScanMatcher::initializeKernel(const int size, const float res, const float dmax)
{
  int center = size;
  int dim = 2*size+1;
  _kernel.resize(dim,dim);
  _kernel.fill(dmax);

  for(int j = 0; j <= size; j++)
  {
    for(int i = 0; i <= size; i++)
    {
      float distance = res*sqrt(j*j+i*i);
      if(distance > dmax)
      {
	continue;
      }
      _kernel(i+center,j+center) = distance;
      _kernel(center-i,j+center) = distance;
      _kernel(i+center,center-j) = distance;
      _kernel(center-i,center-j) = distance;
    }
  }
}


void ScanMatcher::integrateScan(const Vector2fVector& ns, const float& val, const Isometry2f& transform)
{
  Matrix2f matrix = transform.rotation();
  float angle = atan2(matrix(1, 0), matrix(0, 0));
  Vector2f offset = transform.translation();
  float c = cos(angle);
  float s = sin(angle);
  float tx = offset.x();
  float ty = offset.y();
  for(Vector2fVector::const_iterator it = ns.begin(); it != ns.end(); ++it)
  {
    Vector2f p = *it;
    float px = p.x();
    float py = p.y();
    Vector2f transformedPoint(c*px - s*py + tx, s*px + c*py + ty);
    Vector2i ip = _scanGrid.world2grid(transformedPoint);
    if(ip.x() >= 0 && ip.x() < _scanGrid.size().x() && ip.y() >= 0 && ip.y() < _scanGrid.size().y())
    {
      _scanGrid.cell(ip) = val;
      _rasterIndices.push_back(ip);
      _rasterCells.push_back(&_scanGrid.cell(ip));
    }
  }
}


void ScanMatcher::saveConvolvedScanAsPPM(ostream& os, bool eq) const
{
  return _convolvedGrid.saveAsPPM(os, eq);
}


void ScanMatcher::saveScanAsPPM(ostream& os, bool eq) const
{
  return _scanGrid.saveAsPPM(os, eq);
}


void ScanMatcher::scanMatch(const ScanMatcher::Vector2fVector& s, const Eigen::Vector3f& ig) {}


void ScanMatcher::subsample(Vector2fVector& dest, const Vector2fVector& src)
{
  Vector2iAccumulatorMap accMap;
  float gridInvResolution = _scanGrid.inverseResolution();
  for(Vector2fVector::const_iterator it = src.begin(); it!=src.end(); ++it)
  {
    const Vector2f& p = *it;
    Vector2i ip(gridInvResolution* p.x(), gridInvResolution* p.y());
    Vector2iAccumulatorMap::iterator ait = accMap.find(ip);
    if(ait == accMap.end())
    {
      PointAccumulator pa;
      pa.add(p);
      accMap.insert(make_pair(ip, pa));
    }
    else
    {
      PointAccumulator& pa = ait->second;
      pa.add(p);
    }
  }
  dest.resize(accMap.size());
  int i = 0;
  for(Vector2iAccumulatorMap::iterator ait=accMap.begin(); ait!=accMap.end(); ++ait)
  {
    dest[i] = ait->second.mean();
    i++;
  }
}
