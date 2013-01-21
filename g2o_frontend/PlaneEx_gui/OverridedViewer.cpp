/*
 * OverridedViewer.cpp
 *
 *  Created on: Nov 27, 2012
 *      Author: malcom
 */

#include "OverridedViewer.h"
#include <math.h>

OverridedViewer::OverridedViewer(QWidget *parent) : QGLViewer(parent)
{
    dataptr=0;
    planes=0;
    this->setSceneRadius(10);
}



void OverridedViewer::draw()
{

    //Disabilito la luce
    glDisable( GL_LIGHTING );
    glDisable( GL_LIGHT0 );
    glDisable( GL_LIGHT1 );


    //drawAxis();
    glPointSize(1.0f);

    //disegno tutti i punti della cloud
    if(planes==0)
    {
        for (int i=0;i<(int)dataptr->size();i++)
        {
            glBegin(GL_POINTS);
            if((*dataptr)[i][2]!=0)
                glVertex3f((*dataptr)[i][0]/1000, (*dataptr)[i][1]/1000, (*dataptr)[i][2]/1000);
            glEnd();


        }
    }
    //disegno i piani estratti dalla procedura
    if(planes==1)
    {

        int planesNUM=planesContainerPTR->size();
        for(int i=0;i<planesNUM;i++)
        {

            glColor3f((*planesContainerPTR)[i].color[0],
                      (*planesContainerPTR)[i].color[1],
                      (*planesContainerPTR)[i].color[2]);

            for(int j=0;j<(*planesContainerPTR)[i].cloud.size();j++)
            {
                glBegin(GL_POINTS);
                glVertex3f(((*planesContainerPTR)[i].cloud[j].x),
                           ((*planesContainerPTR)[i].cloud[j].y),
                           ((*planesContainerPTR)[i].cloud[j].z));

                glEnd();

                glBegin(GL_LINES);
                glVertex3f((*planesContainerPTR)[i].com[0],
                           (*planesContainerPTR)[i].com[1],
                           (*planesContainerPTR)[i].com[2]);

                glVertex3f((*planesContainerPTR)[i].a+(*planesContainerPTR)[i].com[0],
                           (*planesContainerPTR)[i].b+(*planesContainerPTR)[i].com[1],
                           (*planesContainerPTR)[i].c+(*planesContainerPTR)[i].com[2]);


                glEnd();
            }

            glColor3f(1,1,1);
        }
    }


    /*
    for(int i=0;i<(int)this->nearVector->size();i++)
    {

    }
     */
}

void OverridedViewer::getDataPointer(std::vector<Vec3f> * thePtr)
{
    dataptr=thePtr;
}

