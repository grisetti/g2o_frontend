#ifndef SCANMATCHER_H
#define SCANMATCHER_H


#include "g2o_frontend/sensor_data/laser_robot_data.h"

#include "matcher.h"
#include "../structures/gridmap.h"



struct PointAccumulator
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    PointAccumulator()
    {
        _acc.setZero();
        _count = 0;
    }

    inline int count() const {return _count;}
  
    inline Eigen::Vector2f mean() const
    {
        if(_count)
        {
            return _acc * (1. / (float) _count);
        }
        return _acc;
    }

    inline void add(const Eigen::Vector2f& p)
    {
        _acc += p;
        _count ++;
    }
  
    inline void remove(const Eigen::Vector2f& p)
    {
        if(_count==0)
        {
            return;
        }
        _acc -= p;
        _count --;
    }

    Eigen::Vector2f  _acc;
    int _count;
};


struct Vector2iComparator
{
    inline bool operator()(const Eigen::Vector2i& v1, const Eigen::Vector2i& v2)
    {
        bool condition = ((v1.x() < v2.x()) || (((v1.x() == v2.x()) && (v1.y() < v2.y()))));
        if(condition)
        {
            return true;
        }
        return false;
    }
};
typedef std::map<Eigen::Vector2i, PointAccumulator, Vector2iComparator> Vector2iAccumulatorMap;


class ScanMatcherResult : public MatcherResult
{
    friend class ScanMatcher;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual float matchingScore() const;
    virtual ~ScanMatcherResult();
};


class ScanMatcher : public Matcher
{
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
    typedef std::vector<Eigen::Vector2i, Eigen::aligned_allocator<Eigen::Vector2i> > Vector2iVector;
    typedef std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f> > Vector2fVector;
    typedef Eigen::Matrix<char, Eigen::Dynamic, Eigen::Dynamic> MatrixXChar;
    typedef _GridMap<char> CharGrid;
    typedef std::vector<char*> CellPointersVector;
    typedef std::vector<CellPointersVector*> GridPointer;
    
    ScanMatcher(const float& resolution, const float& radius, const int& kernelSize, const float& kernelMaxValue, const int _gridKScale = 128);
    ScanMatcher(const CharGrid& fg, const int& kernelSize, const float& kernelMaxValue, const int _gridKScale = 128);

    virtual ~ScanMatcher();


    //! function inherited from the super-classes Matcher, compatible with g2o
    //! represention of the graph
    //! @param ref: pointer to the vertex whose data is to use as reference
    //! @param curr: pointer to the vertex whose data is to be matched against the reference
    virtual void match(g2o::OptimizableGraph::Vertex* ref, g2o::OptimizableGraph::Vertex* curr);
    

    void clear();
    void convolveScan(const Vector2fVector& ns, const Eigen::Isometry2f& transform = Eigen::Isometry2f::Identity());
    void integrateMap(const Vector2fVector& ns, const float& val = 1., const Eigen::Isometry2f& transform = Eigen::Isometry2f::Identity());
    void integrateScan(const Vector2fVector& ns, const float& val = 1., const Eigen::Isometry2f& transform = Eigen::Isometry2f::Identity());
    void resizeGrids(const float& resolution, const float& radius, const int& kernelSize, const float& kernelMaxValue, const int _gridKScale = 128);
    void saveConvolvedScanAsPPM(std::ostream& os, bool eq) const;
    void saveScanAsPPM(std::ostream& os, bool eq) const;
    void subsample(Vector2fVector& dest, const Vector2fVector& src);


    inline Vector2iVector& getConvolvedIndices() { return _convolvedIndices; }
    inline const Vector2iVector& getConvolvedIndices() const { return _convolvedIndices; }
    
    inline Vector2iVector& getRasterIndices() { return _rasterIndices; }
    inline const Vector2iVector& getRasterIndices() const { return _rasterIndices; }
    
    inline CharGrid& getScanGrid() { return _scanGrid; }
    inline const CharGrid& getScanGrid() const { return _scanGrid; }
    
    inline CharGrid& getConvolvedGrid() { return _convolvedGrid; }
    inline const CharGrid& getConvolvedGrid() const { return _convolvedGrid; }

    inline int& getGridKScale() { return _gridKScale; }
    inline const int& getGridKScale() const { return _gridKScale; }

    inline void popFirstScan() { _scanContainer.erase(_scanContainer.begin(), _scanContainer.begin()+1); }

  protected:
    void convolveGrid(const CharGrid& g);
    void initializeKernel(const int size, const float res, const float dmax);
    void initializeCharKernel(const int size, const float res, const int dmax);

    CharGrid _scanGrid;
    CharGrid _convolvedGrid;
    MatrixXChar _kernel;

    CellPointersVector _rasterCells;
    CellPointersVector _convolvedCells;
    GridPointer _scanContainer;
    GridPointer _convolvedContainer;

    Vector2iVector _rasterIndices;
    Vector2iVector _convolvedIndices;

    int _gridKScale;
    float _kernelRange;
};

#endif // SCANMATCHER_H
