#include "alignment_horn2d.h"
#include "alignment_horn3d.h"
#include "alignment_se2.h"
#include "alignment_se3.h"
#include "alignment_line3d_linear.h"
#include "alignment_line2d_linear.h"
#include "alignment_plane_linear.h"
#include "g2o_frontend/basemath/bm_se2.h"
#include "g2o_frontend/basemath/bm_se3.h"
#include "g2o/types/slam3d/isometry3d_mappings.h"
#include <iostream>
#include <fstream>
#include <cstdlib>
using namespace Eigen;
using namespace g2o;
using namespace g2o_frontend;
using namespace std;
using namespace Slam3dAddons;


template <typename TypeDomain_, int dimension_>
struct EuclideanMapping{
    typedef TypeDomain_ TypeDomain;
    typedef typename Eigen::Matrix<double, dimension_, 1> VectorType;
    virtual int dimension() const {return dimension_;}
    virtual TypeDomain fromVector(const VectorType& v) const =  0;
    virtual VectorType toVector(const TypeDomain& t) const = 0;
};

template <int dimension_>
struct VectorMapping : public EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>{
    typedef typename EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>::TypeDomain TypeDomain;
    typedef typename EuclideanMapping<Eigen::Matrix<double, dimension_, 1>, dimension_>::VectorType VectorType;
    virtual TypeDomain fromVector(const VectorType& v) const {return v;}
    virtual VectorType toVector(const TypeDomain& t) const {return t;}
};

struct PlaneMapping: public EuclideanMapping<Plane3D,4>{
    typedef typename EuclideanMapping<Plane3D, 4>::TypeDomain TypeDomain;
    typedef typename EuclideanMapping<Plane3D, 4>::VectorType VectorType;
    virtual TypeDomain fromVector(const VectorType& v) const {
        Plane3D l(v);
        //l.normalize(); //already normalized
        return l;
    }
    virtual VectorType toVector(const TypeDomain& t) const {
        return t.toVector();
    }
};



template <typename MappingType, typename RansacType, typename EdgeCorrespondenceType>
bool testAligner(typename RansacType::TransformType& result,
                int nPoints,
                const typename RansacType::TransformType& transform,
                const std::vector<double>& scales,
                const std::vector<double>& offsets,
                const std::vector<double>& noises,
                const std::vector<double>& omegas,
                CorrespondenceValidatorPtrVector& validators,//no
                double outlierFraction=0.0,
                bool debug = false){

    typedef typename RansacType::AlignerType AlignerType;
    typedef typename RansacType::PointVertexType PointVertexType;
    typedef typename RansacType::PointEstimateType PointEstimateType;
    typedef typename RansacType::TransformType TransformType;
    typedef typename MappingType::VectorType VectorType;

    OptimizableGraph graph;

    CorrespondenceVector correspondences;
    IndexVector indices(nPoints, 0);
    MappingType mapping;
    assert(scales.size()==mapping.dimension());
    double zeroVec[100];
    std::fill(zeroVec, zeroVec+100, 0);
    ofstream os1;
    ofstream os2;
    if (debug) {
        os1.open("L1.dat");
        os2.open("L2.dat");
    }
    for (int i=0; i<nPoints; i++){

        VectorType randPoint;
        VectorType randPoint2;
        VectorType noisePoint;
        for(size_t k=0; k<scales.size(); k++){
            randPoint[k]=scales[k]*drand48()+offsets[k];
            randPoint2[k]=scales[k]*drand48()+offsets[k];
            noisePoint[k]=noises[k]*(drand48()-.5);
        }
        PointVertexType* v1=new PointVertexType();
       v1->setEstimate(mapping.fromVector(randPoint));
        v1->setId(i);
        graph.addVertex(v1);

        PointVertexType* v2 = new PointVertexType();
        if (drand48()>outlierFraction | 1) {
            //PointEstimateType v2est=transform * v1->estimate();
            Plane3D v2est=transform * v1->estimate();
            Vector4d coso=v2est.toVector();
            v2->setEstimate(mapping.fromVector(coso));

        } else {
            PointEstimateType v2est=mapping.fromVector(randPoint2);
            v2->setEstimate(v2est);
        }
        v2->setId(i+nPoints);
        graph.addVertex(v2);
        v2->updateCache();

        if (debug) {
            VectorType e1=mapping.toVector(v1->estimate());
            VectorType e2=mapping.toVector(v2->estimate());
            for (int i =0; i<mapping.dimension(); i++){
                os1 << e1[i] << " " ;
                os2 << e2[i] << " " ;
            }

            os1 << endl;
            os2 << endl;
        }
        EdgeCorrespondenceType* edge = new EdgeCorrespondenceType();
        edge->setMeasurementData(zeroVec);
        typename EdgeCorrespondenceType::InformationType info =EdgeCorrespondenceType::InformationType::Identity();
        for (size_t k=0; i<edge->dimension() && k<omegas.size(); k++){
            info(k,k) = omegas[k];
        }
        edge->setInformation(info);
        edge->vertices()[0]=v1;
        edge->vertices()[1]=v2;
        graph.addEdge(edge);
        Correspondence c(edge,100);
        correspondences.push_back(c);
        indices[i]=i;
    }

    RansacType ransac;
    typename RansacType::AlignerType aligner;

    IndexVector indecess(correspondences.size());

    for(int i=0;i<correspondences.size();i++)
    {
        indecess[i]=i;
    }

    return aligner(result, correspondences, indecess);
}

template <typename MappingType, typename RansacType, typename EdgeCorrespondenceType>
bool RansacFromGraph(typename RansacType::TransformType& result,const std::vector<double>& omegas){

    typedef typename RansacType::AlignerType AlignerType;
    typedef typename RansacType::PointVertexType PointVertexType;
    typedef typename RansacType::PointEstimateType PointEstimateType;
    typedef typename RansacType::TransformType TransformType;
    typedef typename MappingType::VectorType VectorType;

    OptimizableGraph graph;
    OptimizableGraph graphToBeRead;

    graphToBeRead.load("cani.g2o");
    CorrespondenceVector correspondences;
    IndexVector indices(3, 0);
    MappingType mapping;
    double zeroVec[100];
    std::fill(zeroVec, zeroVec+100, 0);
    OptimizableGraph::Vertex* _v1=graphToBeRead.vertex(9);

    VertexSE3* v1=dynamic_cast<VertexSE3*>(_v1);
    cerr << v1<<endl;
    //SEGFAULT
    OptimizableGraph::EdgeSet e1 = v1->edges();
    cerr << "Here 4"<<endl;
    OptimizableGraph::Vertex* _v2=graphToBeRead.vertex(10);
    VertexSE3* v2=dynamic_cast<VertexSE3*>(_v2);
    OptimizableGraph::EdgeSet e2 = v2->edges();
    int k=0;
    for (HyperGraph::EdgeSet::iterator it = e1.begin(); it!=e1.end() & k<3; it++)
    {
        HyperGraph::EdgeSet::iterator it2 = e2.begin();

        HyperGraph::Edge* _e = *it;
        HyperGraph::Edge* _e2 = *it2;

        PointVertexType* vv=new PointVertexType();
        const EdgeSE3PlaneSensorCalib* ePlane =dynamic_cast<const EdgeSE3PlaneSensorCalib*>(_e);
        if(ePlane)
        {

            const Plane3D thePlane= ePlane->measurement();

            vv->setEstimate(thePlane);
            graph.addVertex(vv);
        }

        PointVertexType* vv2=new PointVertexType();
        const EdgeSE3PlaneSensorCalib* ePlane2 =dynamic_cast<const EdgeSE3PlaneSensorCalib*>(_e);
        if(ePlane2)
        {

            const Plane3D thePlane2= ePlane2->measurement();

            vv2->setEstimate(thePlane2);
             graph.addVertex(vv2);
        }

        EdgeCorrespondenceType* edge = new EdgeCorrespondenceType();
        edge->setMeasurementData(zeroVec);
        typename EdgeCorrespondenceType::InformationType info =EdgeCorrespondenceType::InformationType::Identity();
        for (size_t j=0; j<edge->dimension() && k<omegas.size(); j++){
            info(j,j) = omegas[j];
        }
        edge->setInformation(info);
        edge->vertices()[0]=vv;
        edge->vertices()[1]=vv2;
        graph.addEdge(edge);
        Correspondence c(edge,100);
        correspondences.push_back(c);
        indices[k]=k;

        k++;
    }


    RansacType ransac;
    //ransac.correspondenceValidators()=validators;
    ransac.setCorrespondences(correspondences);
    ransac.setMaxIterations(1000);
    ransac.setInlierErrorThreshold(1.);
    ransac.setInlierStopFraction(0.5);
    std::vector<int> tmp;
    return ransac(result, tmp,0);
}



int main(int , char** ){

    if(1)
    { // Plane
        cerr << "*************** TEST Plane3D  *************** " <<endl;
        std::vector<double> scales;
        std::vector<double> offsets;
        std::vector<double> noises;
        std::vector<double> omegas;

        // translational part
        for (int i=0; i<3; i++){
            scales.push_back(100);
            offsets.push_back(50);
            noises.push_back(0);
            omegas.push_back(1);
        }
        // rotational part
        for (int i=0; i<3; i++){
            scales.push_back(2);
            offsets.push_back(1);
            noises.push_back(0);
            omegas.push_back(1);
        }

        Vector6d _t;
        _t << 1, 5, 6, .5, .2, .3;
        Isometry3d t0=g2o::internal::fromVectorMQT(_t);

        Isometry3d tresult;
        tresult.setIdentity();

        CorrespondenceValidatorPtrVector validators;
        bool result = RansacFromGraph<PlaneMapping, RansacPlaneLinear, EdgePlane>(tresult,omegas);

        /*
        if (1){
            cerr << "ground truth: " <<endl;
            cerr << g2o::internal::toVectorMQT(t0) << endl;

            cerr << "transform found: " <<endl;
            cerr << g2o::internal::toVectorMQT(tresult) << endl;

            cerr << "Errore: "<<endl;
            cerr << g2o::internal::toVectorMQT(tresult*t0)<< endl<< endl<< endl;
            cerr << (tresult*t0).matrix()<< endl<< endl;

        } else {
            cerr << "unable to find a transform" << endl;
        }
        */

    }


}
