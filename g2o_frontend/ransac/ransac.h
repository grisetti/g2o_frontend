#ifndef _G2O_FRONTEND_RANSAC_H_
#define _G2O_FRONTEND_RANSAC_H_
#include <vector>
#include <algorithm>
#include <assert.h>
#include <Eigen/Geometry>
#include "g2o/core/optimizable_graph.h"

#include <iostream>

namespace g2o_frontend {

using namespace std;

struct Correspondence {
    Correspondence(g2o::OptimizableGraph::Edge* edge_, double score_=0):  _edge(edge_), _score(score_){}
    inline g2o::OptimizableGraph::Edge* edge() {return _edge;}
    inline const g2o::OptimizableGraph::Edge* edge() const {return _edge;}
    inline double score() const {return _score;}
    inline bool operator<(const Correspondence& c2) const { return score()>c2.score(); }  // sorts the corresopondences is ascending order
protected:
    g2o::OptimizableGraph::Edge* _edge;
    double _score;
};

typedef std::vector<Correspondence> CorrespondenceVector;
typedef std::vector<int> IndexVector;

class CorrespondenceValidator{
public:
    CorrespondenceValidator(int minimalSetSize_):_minimalSetSize(minimalSetSize_){}
    virtual bool operator()(const CorrespondenceVector& correspondences, const IndexVector& indices, int k) = 0;
    virtual ~CorrespondenceValidator();
    inline int minimalSetSize() const {return _minimalSetSize;}
protected:
    int _minimalSetSize;
};

typedef std::vector<CorrespondenceValidator*> CorrespondenceValidatorPtrVector;


template <typename TransformType_, typename PointVertexType_>
class AlignmentAlgorithm{
public:
    typedef TransformType_ TransformType;
    typedef PointVertexType_ PointVertexType;
    typedef typename PointVertexType_::EstimateType PointEstimateType;
    
    AlignmentAlgorithm(int minimalSetSize_) {_minimalSetSize = minimalSetSize_;}
    virtual bool operator()(TransformType& transform, const CorrespondenceVector& correspondences, const IndexVector& indices) = 0;
    inline int minimalSetSize() const  {return _minimalSetSize;}
protected:
    int _minimalSetSize;
};

//typedef AlignmentAlgorithm<g2o::SE2,g2o::VertexLine2D>  AlignmentAlgorithmSE2Line2D;
//typedef AlignmentAlgorithm<Eigen::Isometry3d,Slam3dAddons::VertexLine3D>   AlignmentAlgorithmSE3Line3D;


class BaseGeneralizedRansac{
public:
    BaseGeneralizedRansac(int _minimalSetSize);
    ~BaseGeneralizedRansac();

    void setCorrespondences(const CorrespondenceVector& correspondences_);
    inline const CorrespondenceVector& correspondences() const {return _correspondences;}
    bool validateCorrespondences(int k);
    inline CorrespondenceValidatorPtrVector& correspondenceValidators() {return _correspondenceValidators;}
    inline const CorrespondenceValidatorPtrVector& correspondenceValidators() const {return _correspondenceValidators;}
    inline void setInlierErrorThreshold(double inlierErrorThreshold_) {_inlierErrorTheshold = inlierErrorThreshold_;}
    inline double inlierErrorThreshold() const {return _inlierErrorTheshold;}
    inline void setInlierStopFraction(double inlierStopFraction_) {_inlierStopFraction = inlierStopFraction_;}
    inline double inlierStopFraction() const {return _inlierStopFraction;}
    inline const std::vector<double> errors() const { return _errors;}
    inline int minimalSetSize() const {return _minimalSetSize;}
    inline const IndexVector getInliersIndeces() const {return _indices;}
    int maxIterations() const {return _maxIterations;}
    void setMaxIterations(double maxIterations_) { _maxIterations = maxIterations_;}

protected:
    bool _init();
    bool _cleanup();

    int _minimalSetSize;
    int _maxIterations;
    IndexVector _indices;
    CorrespondenceVector _correspondences;
    CorrespondenceValidatorPtrVector _correspondenceValidators;
    double _inlierErrorTheshold;
    double _inlierStopFraction;
    std::vector<double> _errors;
    std::vector<g2o::OptimizableGraph::Vertex*> _vertices1;
    std::vector<g2o::OptimizableGraph::Vertex*> _vertices2;
    bool _verticesPushed;
};

template <typename AlignmentAlgorithmType>
class GeneralizedRansac : public BaseGeneralizedRansac{
public:
    typedef AlignmentAlgorithmType AlignerType;
    typedef typename AlignmentAlgorithmType::TransformType TransformType;
    typedef typename AlignmentAlgorithmType::PointVertexType PointVertexType;
    typedef typename AlignmentAlgorithmType::PointVertexType::EstimateType PointEstimateType;

    GeneralizedRansac(int minimalSetSize_) :BaseGeneralizedRansac(minimalSetSize_){
    }


    inline AlignmentAlgorithmType& alignmentAlgorithm() {return _alignmentAlgorithm;}
    inline const AlignmentAlgorithmType& alignmentAlgorithm() const {return _alignmentAlgorithm;}

    ostream& pindex(ostream& os, const std::vector<int> v, size_t k) {
        for (size_t i=0; i<=k && i<v.size(); i++){
            os << v[i] << " " ;
        }
        os << endl;
        return os;
    }

    bool computeMinimalSet(TransformType& t, int k){
        // base step
        bool transformFound = false;
        int maxIndex =_correspondences.size();
        while (_indices[k]<maxIndex && ! transformFound){
            bool validated = validateCorrespondences(k);
            if (! validated) {
                _indices[k] ++;
                continue;
            } else {
                if (k==minimalSetSize()-1){
                    transformFound = _alignmentAlgorithm(t,_correspondences,_indices);
                    //cerr << "Inner Iteration (" << k << ") : ";
                    // pindex(cerr, _indices, k);
                    // cerr << endl;
                    _indices[k]++;
                } else {
                    if (_indices[k+1]<_indices[k])
                        _indices[k+1]=_indices[k]+1;

                    transformFound = computeMinimalSet(t,k+1);

                    if(_indices[k+1]>maxIndex-((int)_indices.size()-k)){
                        _indices[k]++;
                        _indices[k+1]=_indices[k]+1;
                    }
                }
            }
        }
        return transformFound;
    }

    //mal
    bool operator()(TransformType& treturn, std::vector<int>& inliers_, bool debug = false){
        if ((int)_correspondences.size()<minimalSetSize())
            return false;
        if (!_init())
            assert(0 && "_ERROR_IN_INITIALIZATION_");

        std::vector<int> bestInliers;
        bestInliers.reserve(_correspondences.size());
        double bestError=std::numeric_limits<double>::max();
        TransformType bestTransform = treturn;

        bool transformFound = false;
        for (int i=0; i<_maxIterations && !(_indices[0]>(int)_correspondences.size()-(int)_indices.size());  i++){
            TransformType t;
            if(debug)
                cerr << "iteration: " << i << endl;
            if (! computeMinimalSet(t,0)) {
                if(debug)
                    cerr << "FAIL" << endl;
                continue;
            }
            if(debug)
                cerr << "OK" << endl;
            std::vector<int> inliers;
            inliers.reserve(_correspondences.size());
            std::vector<double> currentErrors(_correspondences.size());
            double error = 0;

            // a transform has been found, now we need to apply it to all points to determine if they are inliers
            for(size_t k=0; k<_correspondences.size(); k++){
                Correspondence& c=_correspondences[k];
                g2o::OptimizableGraph::Edge* e=c.edge();
                PointVertexType* v1=static_cast<PointVertexType*>(e->vertex(0));
                PointVertexType* v2=static_cast<PointVertexType*>(e->vertex(1));
                PointEstimateType ebackup = v2->estimate();
                v2->setEstimate(t*ebackup);
                e->computeError();
                currentErrors[k] = e->chi2();
                //cerr << "e: " << e->chi2() << endl;
                if (e->chi2()<_inlierErrorTheshold){
                    if (debug) {
		      cerr << "**************** INLIER ****************" << endl;                        cerr << endl << "v1 " << v1->id() << " ";
                        v1->write(cerr);
                        cerr << endl;
                        v2->setEstimate(ebackup);
                        cerr <<  "v2 " << v2->id() << " ";
                        v2->write(cerr);
                        cerr << endl;
                        v2->setEstimate(t*ebackup);
                        cerr << "remappedV2 ";
                        v2->write(cerr);
                        cerr << endl;
                        cerr << "chi2: " << e->chi2() << endl;
                        cerr << "error: " << error << endl;
                    }
                    inliers.push_back(k);
                    error+=e->chi2();
                }
                v2->setEstimate(ebackup);
            }
            if(debug)
                cerr << "transformFound:" << (int) transformFound << endl;
            if (debug)
                cerr << "inliers:" << inliers.size() << endl;
            if ((int)inliers.size()<minimalSetSize()) {
                if(debug)
                cerr << "too few inliers: " << (int)inliers.size() <<  endl;
                continue;
            }
            if(debug)
            cerr << "error:" << error/inliers.size() << endl;

            if (inliers.size()>bestInliers.size()){
                if(debug)
		  cerr << "enough inliers: " << (int)inliers.size() <<  endl;
                double currentError = error/inliers.size();
                if (currentError<bestError){
                    if(debug)
                        cerr << "good error: " << currentError <<  endl;
                    bestError= currentError;
                    bestInliers = inliers;
		    _errors = currentErrors;
                    bestTransform = t;
                    transformFound = true;

                    //mal
                    inliers_=bestInliers;
		    //debug martina
		    //cerr << "AAAAA" << endl;
		    for (size_t i = 0; i<inliers.size(); i++){
		      cerr << inliers[i] << " ";
		    }
		    cerr << endl;
		    //cerr << "BBBBB" << endl;
		    for (size_t i = 0; i<inliers.size(); i++){
		      int idx = inliers[i];
		      cerr << _errors[idx] << " ";
		    }
		    cerr << endl;
		    //cerr << "CCCCC" << endl;
		    for (size_t i = 0; i<inliers.size(); i++){
		      int idx = inliers[i];	
		      Correspondence& c=_correspondences[idx];
		      g2o::OptimizableGraph::Edge* e=c.edge();
		      PointVertexType* v1=static_cast<PointVertexType*>(e->vertex(0));
		      PointVertexType* v2=static_cast<PointVertexType*>(e->vertex(1));
		      cerr << "inliers are: " << "(" << idx << ","<< e << ","<< v1->id() << "," << v2->id() << "), ";
		    }
		    cerr << endl;
                }
                if ((double)bestInliers.size()/(double)_correspondences.size() > _inlierStopFraction){
                    transformFound = true;
                    if(debug)
                        cerr << "excellent inlier fraction: "
                             << 1e2 *(double)inliers.size()/(double)_correspondences.size() <<"%"  <<endl;
                    break;
                }
            }
        }
        if (transformFound){
	  _alignmentAlgorithm(bestTransform,_correspondences,bestInliers);
        }
        treturn = bestTransform;
        if (!_cleanup())
            assert(0 && "_ERROR_IN_CLEANUP_");
        return transformFound;
    }

protected:
    AlignmentAlgorithmType _alignmentAlgorithm;
};

}

#endif
