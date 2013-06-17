#include <stdio.h>

#include "scan_matcher.h"


using namespace std;
using namespace Eigen;

namespace match_this
{

float ScanMatcherResult::matchingScore() const
{
    return _matchingScore;
}


ScanMatcherResult::~ScanMatcherResult() {}


ScanMatcher::ScanMatcher(const float& resolution, const float& radius, const int& kernelSize,
                         const float& kernelMaxValue, const int kscale)
{
    _kernelRange = kernelMaxValue;
    _gridKScale = kscale;

    int size = (int) 2*(radius/resolution);
    this->_scanGrid = CharGrid(Vector2i(size, size), Vector2f(-radius, -radius), resolution, char(0));
    this->_convolvedGrid = CharGrid(Vector2i(size, size), Vector2f(-radius, -radius), resolution, (char) _kernelRange);
    this->initializeKernel(kernelSize, resolution, _kernelRange);

    //   _rasterCells.reserve(size);
    //   _convolvedCells.reserve(size);
    //   _rasterIndices.reserve(size);
    //   _convolvedIndices.reserve(size);
}


ScanMatcher::ScanMatcher(const CharGrid& inputGrid, const int& kernelSize,
                         const float& kernelMaxValue, const int kscale)
{
    _kernelRange = kernelMaxValue;
    _gridKScale = kscale;

    this->_scanGrid = inputGrid;
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

    this->_convolvedGrid = CharGrid(inputGrid.size(), inputGrid.lowerLeft(), inputGrid.resolution(), (char) _kernelRange);
    this->initializeKernel(kernelSize, inputGrid.resolution(), _kernelRange);
    this->convolveGrid(_scanGrid);
}


void ScanMatcher::resizeGrids(const float &resolution, const float &radius, const int &kernelSize,
                              const float &kernelMaxValue, const int kscale)
{
    _kernelRange = kernelMaxValue;
    _gridKScale = kscale;

    int size = (int) 2*(radius/resolution);
    this->_scanGrid = CharGrid(Vector2i(size, size), Vector2f(-radius, -radius), resolution, char(0));
    this->_convolvedGrid = CharGrid(Vector2i(size, size), Vector2f(-radius, -radius), resolution, (char) _kernelRange);
    this->initializeKernel(kernelSize, resolution, _kernelRange);
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
        *(_convolvedCells[i]) = (char) _kernelRange;
    }
    _convolvedCells.clear();
    _convolvedIndices.clear();

    int sizeRC = _rasterCells.size();
    for(int j = 0; j < sizeRC; ++j)
    {
        *(_rasterCells[j]) = char(0);
    }

    _rasterCells.clear();
    _rasterIndices.clear();
}


void ScanMatcher::convolveGrid(const CharGrid& g)
{
    const char* ker = _kernel.data();
    int kRows = _kernel.rows();
    int kCols = _kernel.cols();
    Vector2i gridSize = _convolvedGrid.size();
    int oRows = gridSize.x();
    int oCols = gridSize.y();
    int center = (kRows-1)/2;

    Vector2iVector gRasterIndices = this->getRasterIndices();
    for(Vector2iVector::const_iterator it = gRasterIndices.begin(); it != gRasterIndices.end(); ++it)
    {
        Vector2i ip = *it;
        int r = ip.x();
        int c = ip.y();
        for(int i = 0; i < kRows; ++i)
        {
            int iOut = r+i-center;
            bool condition1 = ((iOut >= 0) && (iOut < oRows));
            if(condition1)
            {
                for(int j = 0; j < kCols; ++j)
                {
                    int jOut = c+j-center;
                    bool condition2 = ((jOut >= 0) && (jOut < oCols));
                    if(condition2)
                    {
                        char& v = _convolvedGrid.cell(iOut, jOut);
                        const char& k = ker[j*kRows+i];
                        v = (k < v) ? k : v;
                        Vector2i currentIndices(iOut, jOut);
                        _convolvedIndices.push_back(currentIndices);
                        _convolvedCells.push_back(&_convolvedGrid.cell(currentIndices));
                    }
                }
            }
        }
    }
}


void ScanMatcher::convolveScan(const Vector2fVector& ns, const Isometry2f& transform)
{
    const char* ker = _kernel.data();
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
            bool condition1 = ((iOut >= 0) && (iOut < oRows));
            if(condition1)
            {
                for(int j = 0; j < kCols; ++j)
                {
                    int jOut = c+j-center;
                    bool condition2 = ((jOut >= 0) && (jOut < oCols));
                    if(condition2)
                    {
                        char& v = _convolvedGrid.cell(iOut, jOut);
                        const char& k = ker[j*kRows+i];
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
    int dim = (2*size)+1;
    _kernel.resize(dim,dim);

    int K1 = res * _gridKScale;
    int K2 = dmax * _gridKScale;
    _kernel.fill((char) K2);

    for(int j = 0; j <= size; ++j)
    {
        for(int i = 0; i <= size; ++i)
        {
            char distance = K1*sqrt(j*j+i*i);
            if(distance > K2)
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


void ScanMatcher::match(g2o::OptimizableGraph::Vertex *ref, g2o::OptimizableGraph::Vertex *curr) {}


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
        //    if(ip.x() >= 0 && ip.x() < _scanGrid.size().x() && ip.y() >= 0 && ip.y() < _scanGrid.size().y())
        //    {
        _scanGrid.cell(ip) = (char) val;
        _rasterIndices.push_back(ip);
        _rasterCells.push_back(&_scanGrid.cell(ip));
        //    }
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


void ScanMatcher::subsample(Vector2fVector& dest, const Vector2fVector& src)
{
    Vector2iAccumulatorMap accMap;
    float gridInvResolution = _scanGrid.inverseResolution();
    for(Vector2fVector::const_iterator it = src.begin(); it!=src.end(); ++it)
    {
        const Vector2f& p = *it;
        Vector2i ip(gridInvResolution*p.x(), gridInvResolution*p.y());
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
}
