#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <deque>


#include "voronoi_edge.h"
#include "voronoi_vertex.h"


using namespace std;
using namespace Eigen;


struct Comparator
{
    bool operator() (const Vector2i& lhs, const Vector2i& rhs) const
    {
        if(lhs.x() < rhs.x())
        {
            return true;
        }
        else if((lhs.x() == rhs.x()) && (lhs.y() < rhs.y()))
        {
            return true;
        }
        else
        {
            return false;
        }
    }
};


//typedef vector<VoronoiEdge> EdgeSet;
typedef deque<VoronoiVertex*> PointersDeque;
typedef map<Vector2i, VoronoiVertex, Comparator> VerticesMap;
typedef pair<Vector2i, VoronoiVertex> VoronoiPair;



void loadPPM(istream& is, MatrixXf*& map, VerticesMap& vertices)
{
    string tag;
    is >> tag;
    if (tag!="P6")
    {
        cerr << "Awaiting 'P6' in ppm header, found " << tag << endl;
        exit(-1);
    }

    int sizeX, sizeY;

    while (is.peek()==' ' || is.peek()=='\n') is.ignore();
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> sizeX;
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> sizeY;
    while (is.peek()=='#') is.ignore(255, '\n');
    is >> tag;
    if(tag!="255")
    {
        cerr << "Awaiting '255' in ppm header, found " << tag << endl;
        exit(-1);
    }

    map = new MatrixXf(sizeX, sizeY);

    int counter = 0;
    for(int y = sizeY-1; y >= 0; y--)
    {
        for(int x = 0; x < sizeX; x++)
        {
            float c = is.get();     // HACK TO HANDLE
            c = is.get();           // R, G, B
            c = is.get();           // CHANNELS
            (*map)(x, y) = c;
            VoronoiVertex v(c, Vector2i(x, y));
            vertices.insert(VoronoiPair(v.position(), v));
            counter++;
        }
    }
    cout << "Acceptable points: " << counter << endl;
}


void savePPM(const char *filename, MatrixXf& map)
{
    FILE* F = fopen(filename, "w");
    if(!F)
    {
        cerr << "could not open 'result.pgm' for writing!" << endl;
        return;
    }

    int sizeX = map.rows();
    int sizeY = map.cols();

    fprintf(F, "P6\n");
    fprintf(F, "%d %d 255\n", sizeX, sizeY);

    for(int y = sizeY-1; y >= 0; y--)
    {
        for(int x = 0; x < sizeX; ++x)
        {
            fputc(map(x, y), F);
            fputc(map(x, y), F);
            fputc(map(x, y), F);
        }
    }
    fclose(F);
}


void mapToMatrix(VerticesMap& map, MatrixXf& image)
{
    for(VerticesMap::iterator it = map.begin(); it != map.end(); ++it)
    {
        VoronoiVertex v = it->second;
        if(v.visited())
        {
            image(v.position().x(), v.position().y()) = v.distance();
        }
    }
}


void breadthFirst(VerticesMap& set, PointersDeque& deque, MatrixXi& boolean, MatrixXf& map, int& connected)
{
    int counter = 0;
    for(VerticesMap::iterator it = set.begin(); it != set.end(); ++it)
    {
        VoronoiVertex* first = &(it->second);
        if(boolean(first->position().x(), first->position().y()) == 0)
        {
            deque.push_back(first);
            counter++;
        }
        while(!deque.empty())
        {
            VoronoiVertex* parent = deque.front();
            deque.pop_front();

            int x = parent->position().x();
            int y = parent->position().y();
            boolean(x, y) = 1;
            parent->setVisited();

            int childCounter = 0;
            for(short int r = -1; r < 2; ++r)
            {
                for(short int c = -1; c < 2; ++c)
                {
                    int currentX = x+r;
                    int currentY = y+c;

                    //Out of boundaries
                    if((currentX < 0) || (currentY < 0) || (currentX >= map.rows()) || (currentY >= map.cols()))
                    {
                        continue;
                    }

                    //Checking the parent node
                    if((currentX == x) && (currentY == y))
                    {
                        continue;
                    }

                    if(((map(currentX, currentY)) == 0) || ((boolean(currentX, currentY)) != 0))
                    {
                        continue;
                    }

                    VoronoiVertex child;
                    child.setPosition(currentX, currentY);
                    VerticesMap::iterator pit = set.find(child.position());
                    if(pit != set.end())
                    {
                        VoronoiVertex* v2 = &(pit->second);
                        boolean(currentX, currentY) = 1;
                        v2->setVisited();
                        deque.push_back(v2);
                        VoronoiEdge e1(parent, v2);
                        parent->addToEdgeSet(e1);
                        childCounter++;
                    }
                }
            }
            parent->setOrder(childCounter);
        }
    }
    connected = counter;
}


int main(int argc, char** argv)
{
    if(argc < 2 || argc > 3)
    {
        cerr << "usage: " << argv[0] <<" <ppm map> " << endl;
        exit(-1);
    }

    ifstream is(argv[1]);
    if(!is)
    {
        cerr << "Could not open map file for reading." << endl;
        exit(-1);
    }

    VerticesMap verticesMap;
    MatrixXf* voronoiDiagram = new MatrixXf;
    loadPPM(is, voronoiDiagram, verticesMap);

    MatrixXi booleanImage = MatrixXi::Zero(voronoiDiagram->rows(), voronoiDiagram->cols());
    PointersDeque pointersQue;
    int components;
    breadthFirst(verticesMap, pointersQue, booleanImage, *voronoiDiagram, components);

    int maxOrder = -1;
    float maxDistance = -1.0;
    for(VerticesMap::iterator it = verticesMap.begin(); it != verticesMap.end(); ++it)
    {
        VoronoiVertex v = it->second;
        cout << "VERTEX: " << v.position().x() << ", " << v.position().y() << "; #" << v.order() << endl;
        if(v.order() >= maxOrder)
        {
            maxOrder = v.order();
        }
        if(v.distance() >= maxDistance)
        {
            maxDistance = v.distance();
        }
        EdgeSet es = v.edgeSet();
        for(EdgeSet::iterator it = es.begin(); it != es.end(); ++it)
        {
            VoronoiEdge e1 = *it;
            cout << "from: " << e1.from()->position().x() << ", " << e1.from()->position().y() << " to: " << e1.to()->position().x() << ", " << e1.to()->position().y() << endl;
        }
    }
    cout << "Max distance is: " << maxDistance << endl;
    cout << "Max order is: " << maxOrder << endl;
    cout << "NUMBER OF CONNECTED COMPONENTS: " << components << endl;

    MatrixXf mappa2 = MatrixXf::Zero(voronoiDiagram->rows(), voronoiDiagram->cols());
    mapToMatrix(verticesMap, mappa2);
    savePPM("output2.ppm", mappa2);

    exit(0);
}
