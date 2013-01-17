#ifndef __SCAN_MATCHER_H__
#define __SCAN_MATCHER_H__

#include "matcher.h"
#include "g2o_frontend/matcher/structures/gridmap.h"



struct PointAccumulator
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
  PointAccumulator()
  {
    _acc.setZero();
    _count = 0;
  }
  
  int count() const {return _count;}
  
  Eigen::Vector2f mean() const
  { 
    if(_count)
      return _acc * (1. / (float) _count);
    return _acc;
  }

  void add(const Eigen::Vector2f& p)
  {
    _acc += p;
    _count ++;
  }
  
  void remove(const Eigen::Vector2f& p)
  {
    if (_count==0)
      return;
    _acc -= p;
    _count --;
  }

  Eigen::Vector2f  _acc;
  int _count;
};


struct Vector2iComparator
{
  bool operator () (const Eigen::Vector2i& v1, const Eigen::Vector2i& v2)
  {
    if (v1.x() < v2.x() || (v1.x() == v2.x() && v1.y() < v2.y()))
      return true;
    return false;
  }
};
typedef std::map<Eigen::Vector2i, PointAccumulator, Vector2iComparator> Vector2iAccumulatorMap;


class ScanMatcherResult : public MatcherResult
{
  public:
    friend class ScanMatcher;
    virtual const float matchingScore() const;
    virtual ~ScanMatcherResult();
};


class ScanMatcher : public Matcher
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  
    typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;
    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
    typedef Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> MatrixXChar;    
    typedef _GridMap<float> FloatGrid;
    typedef _GridMap<char> CharGrid;

    
    ScanMatcher(const float& resolution, const float& radius, const int& kernelSize, const float& kernelMaxValue);
    ScanMatcher(const _GridMap<float>& g, const int& kernelSize, const float& kernelMaxValue);
    
    virtual ~ScanMatcher();
    virtual void scanMatch(const Vector2fVector& s, const Eigen::Vector3f& ig);
    
    void clear();
    void convolveScan(const Vector2fVector& ns, const Eigen::Isometry2f& transform = Eigen::Isometry2f::Identity());
    void integrateMap(const Vector2fVector& ns, const float& val = 1., const Eigen::Isometry2f& transform = Eigen::Isometry2f::Identity());
    void integrateScan(const Vector2fVector& ns, const float& val = 1., const Eigen::Isometry2f& transform = Eigen::Isometry2f::Identity());
    void saveConvolvedScanAsPPM(std::ostream& os, bool eq) const;
    void saveScanAsPPM(std::ostream& os, bool eq) const;
    void subsample(Vector2fVector& dest, const Vector2fVector& src);
    
    
    inline std::vector<Eigen::Vector2i>& getConvolvedIndices() {return _convolvedIndices;}
    inline const std::vector<Eigen::Vector2i>& getConvolvedIndices() const {return _convolvedIndices;}    
    
    inline std::vector<Eigen::Vector2i>& getRasterIndices() {return _rasterIndices;}
    inline const std::vector<Eigen::Vector2i>& getRasterIndices() const {return _rasterIndices;}
    
    inline FloatGrid& getScanGrid() {return _scanGrid;}
    inline const FloatGrid& getScanGrid() const {return _scanGrid;}
    
  protected:
    void convolveGrid(const _GridMap<float>& g);
    void initializeKernel(const int size, const float res, const float dmax);

    FloatGrid _scanGrid;
    FloatGrid _convolvedGrid;
    
    std::vector<float*> _rasterCells;
    std::vector<float*> _convolvedCells;
    std::vector<Eigen::Vector2i> _rasterIndices;
    std::vector<Eigen::Vector2i> _convolvedIndices;
    
    
    Eigen::MatrixXf _kernel;
};
#endif
