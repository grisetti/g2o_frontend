/*
 * interface.cpp
 *
 *  Created on: Nov 27, 2012
 *      Author: malcom
 */

#include "interface.h"
#include <Eigen/Geometry>
#include "g2o/stuff/timeutil.h"

using namespace Eigen;


extern float voxelParam;
extern int numpointParam;
extern int treshParam;

Vector3d kinectRay(0,0,1);


void ViewerInterface::updateZmin(int val)
{
    cout << "slider 1 val "<<val<<endl;
    slider1value=val;
    this->utils->cleanCloud(slider1value,slider2value,slider3value);
    this->viewer->updateGL();
}

void ViewerInterface::updateZmax(int val)
{
    slider2value=val;
    cout << "slider 2 val "<<val<<endl;
    this->utils->cleanCloud(slider1value,slider2value,slider3value);
    this->viewer->updateGL();
}

void ViewerInterface::updateP(int val)
{
    slider3value=val;
    cout << "slider 3 val "<<(float)val/1000.0f<<endl;
    this->utils->cleanCloud(slider1value,slider2value,slider3value);
    this->viewer->updateGL();
}

//VOXELIZATION
//***********************************************

struct voxelAcc
{
    Vec3f acc;
    int i;

    voxelAcc()
    {
        acc[0]=acc[1]=acc[2]=0;
        i=0;
    };

    void add(Vec3f & v)
    {
        i++;
        acc+=v;
    };

    Vec3f average() const
    {
        float f=1./i;
        return acc*f;
    }
};

struct something
{
    int i[3];
    bool operator < (const something & s) const
    {
        if(i[0]<s.i[0]) return true;
        if(i[0]==s.i[0] && i[1]<s.i[1]) return true;
        if(i[1]==s.i[1] && i[2]<s.i[2]) return true;
        return false;
    }
};
//***********************************************

typedef std::map<something, voxelAcc> accumulatorMap;

//***********************************************


void ViewerInterface::showFull()
{
    accumulatorMap accMap;

    
    float res = voxelParam;
    float ires=1./res;
    
    g2o::ScopeTime t("aa");

    for(int i=0;i<(int)Cloud->size();i++)
    {
        //cout << (*Cloud)[i][0];
        something s;
        res=(float)voxelParam;
	ires=1./res;
        s.i[0]=(int)(*Cloud)[i][0]*ires;
        s.i[1]=(int)(*Cloud)[i][1]*ires;
        s.i[2]=(int)(*Cloud)[i][2]*ires;

        accumulatorMap::iterator it=accMap.find(s);
        if(it==accMap.end())
        {
            voxelAcc vac;
            vac.acc[0]=(*Cloud)[i][0];
            vac.acc[1]=(*Cloud)[i][1];
            vac.acc[2]=(*Cloud)[i][2];
            vac.i=1;
            accMap.insert(make_pair(s,vac));
        }
        else
        {
            voxelAcc & vac = it->second;
            vac.add((*Cloud)[i]);
        }

    }


    cout << "Cloud has  "<<Cloud->size()<<" points"<<endl;
    Cloud->clear();
    for(accumulatorMap::iterator it=accMap.begin();it!=accMap.end();it++ )
    {
        voxelAcc &aMap=it->second;
        Vec3f tmp = aMap.average();
        Cloud->push_back(tmp);


    }
    cout << "voxelized in "<<Cloud->size()<<" points"<<endl;
    cout << "Voxelization finished..."<<endl;
    cout << "Starting plane extraction..."<<endl;


    //ESTRAZIONE DEI PIANI
    pcl::PointCloud<pcl::PointXYZ> pclCLOUD;

    cout << "Populataing a PCL cloud..."<<endl;

    for(int i=0;i<Cloud->size();i++)
    {
        pcl::PointXYZ tmp((*Cloud)[i][0],(*Cloud)[i][1],(*Cloud)[i][2]);
        pclCLOUD.push_back(tmp);
    }

    cout << "Starting Sample consensus procedure..."<<endl;

    //--------------------------------------------------------------------------------
    //DICHIARAZIONI PER L'ESTRAZIONE DEI TANTI PIANI
    const int numpoint=numpointParam;
    const int tresh=treshParam;

    int planesExtracted=0;
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    //------------// Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    //------------// Optional
    seg.setOptimizeCoefficients (true);
    //------------// Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold(tresh);
    //--------------------------------------------------------------------------------
    //Procedure
    seg.setInputCloud (pclCLOUD.makeShared ());

    seg.segment (*inliers, *coefficients);

    Cloud->clear();

    while(
          (inliers->indices.size()!=0) &
          (inliers->indices.size()>numpoint) )
    {


        planesExtracted++;
        plane tmpCLOUD;

        //Istanzio un plane, lo popolo con i punti degli inliers rilevati in precedenza


        const int repj=1000;
        Vec3f com_tmp;
        com_tmp[0]=0;com_tmp[1]=0;com_tmp[2]=0;

        for (size_t i = 0; i < inliers->indices.size (); ++i)
        {
            tmpCLOUD.cloud.push_back(pcl::PointXYZ(
                                         pclCLOUD[inliers->indices[i]].x/repj,
                                         pclCLOUD[inliers->indices[i]].y/repj,
                                         pclCLOUD[inliers->indices[i]].z/repj));

            com_tmp[0]+= pclCLOUD[inliers->indices[i]].x/repj;
            com_tmp[1]+= pclCLOUD[inliers->indices[i]].y/repj;
            com_tmp[2]+= pclCLOUD[inliers->indices[i]].z/repj;
        }

        com_tmp[0]=com_tmp[0]/inliers->indices.size ();
        com_tmp[1]=com_tmp[1]/inliers->indices.size ();
        com_tmp[2]=com_tmp[2]/inliers->indices.size ();

        //Aggiungo alla struttura dati il piano appena trovato
        cout << "[Result] IL PIANO "<<planesExtracted << " HA "<<tmpCLOUD.cloud.size()<< " PUNTI"<<endl;


        cout << "coeffiecienti estratti a:("<<
                coefficients->values[0]<<") b:("<<
                coefficients->values[1]<<") c:("<<
                coefficients->values[2]<<") d:("<<
                coefficients->values[3]<<")"<<endl;


        //controllo se le normali sono coerenti con il punto di vista del kinect



        //assegno i coefficienti del piano estratti da PCL salvandoli alla lista dei piani estratti
        tmpCLOUD.a=coefficients->values[0];
        tmpCLOUD.b=coefficients->values[1];
        tmpCLOUD.c=coefficients->values[2];
        tmpCLOUD.d=coefficients->values[3];

        //salvo il centro di massa
        tmpCLOUD.com=com_tmp;
        Vector3d piano(coefficients->values[0],coefficients->values[1],coefficients->values[2]);
        Vector3d mass(com_tmp[0],com_tmp[1],com_tmp[2]);
        float facingDirection=piano.dot(mass);
        cout << "FACING HAS VALUE OF "<<facingDirection<<endl;
        cout << mass<<endl<<endl;
        cout << piano<<endl;
        if(facingDirection>0)
        {
            cout << "la normale punta in direzione opposta al kinect, va rimappata!"<<endl;
            piano*=-1;
            coefficients->values[0]=piano.coeff(0);
            coefficients->values[1]=piano.coeff(1);
            coefficients->values[2]=piano.coeff(2);
            coefficients->values[3]=piano.coeff(3);

            tmpCLOUD.a=coefficients->values[0];
            tmpCLOUD.b=coefficients->values[1];
            tmpCLOUD.c=coefficients->values[2];
            tmpCLOUD.d=coefficients->values[3];
        }

        this->planes.push_back(tmpCLOUD);
        tmpCLOUD.cloud.clear();

        //Elimino i punti appena rilevati dalla nuvola inizale
        //VECCHIA IMPLEMENTAZIONE
        pcl::PointCloud<pcl::PointXYZ> temporanyCLOUD;

        for(int k=0;k<pclCLOUD.size();k++)
        {
            bool valid=1;
            for (size_t j = 0; j < inliers->indices.size (); ++j)
            {
                if(inliers->indices[j]!=k) valid=1;
                else
                {
                    valid=0;
                    break;
                }
            }
            if(valid)
            {
                temporanyCLOUD.push_back(pclCLOUD[k]);
            }
        }

        pclCLOUD.clear();

        for(int k=0;k<temporanyCLOUD.size();k++)
        {
            pclCLOUD.push_back(temporanyCLOUD[k]);
        }


        /*
        for (size_t p = 0; p < inliers->indices.size (); p++)
        {
            pclCLOUD.erase(pclCLOUD.begin()+inliers->indices[p]);
        }
        pclCLOUD.resize(inliers->indices.size ());
        */

        //Estraggo il piano successivo
        seg.setInputCloud (pclCLOUD.makeShared ());
        seg.segment (*inliers, *coefficients);
    }

    cout << "Found "<<planesExtracted<<" planes"<<endl;
    cout << "Terminating procedure and drawing..."<<endl;

    this->viewer->planes=1;
    this->viewer->planesContainerPTR=&(this->planes);
    this->viewer->updateGL();

}

ViewerInterface::ViewerInterface(CloudUtils * theUtils,std::vector<Vec3f> * theCloud,std::vector<Vec3f> * theFull,QWidget *parent)
{
    setupUi(this);
    QObject::connect(Slider1, SIGNAL(sliderMoved(int)), this, SLOT(updateZmin(int)));
    QObject::connect(Slider2, SIGNAL(sliderMoved(int)), this, SLOT(updateZmax(int)));
    QObject::connect(Slider3, SIGNAL(sliderMoved(int)), this, SLOT(updateP(int)));
    QObject::connect(pushButton, SIGNAL(clicked()), this, SLOT(showFull()));

    Cloud=theCloud;
    Full=theFull;
    utils=theUtils;
    slider1value=0;
    slider2value=0;
    slider3value=0;

}
