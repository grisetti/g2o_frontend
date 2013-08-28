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
        image(v.position().x(), v.position().y()) = v.distance();
    }
}


void maxChildrenCounter(VerticesMap& vertMap, MatrixXf& map)
{
    for(VerticesMap::iterator it = vertMap.begin(); it != vertMap.end(); ++it)
    {
        VoronoiVertex* first = &(it->second);
        int x = first->position().x();
        int y = first->position().y();

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

                if(map(currentX, currentY) == 0)
                {
                    continue;
                }

                childCounter++;
            }
        }
        first->setOrder(childCounter);
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

                    if(((map(currentX, currentY)) == 0) /*|| ((boolean(currentX, currentY)) != 0)*/)
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


    MatrixXf prova = MatrixXf::Zero(6, 6);
    MatrixXi booleanaProva = MatrixXi::Zero(6, 6);
    VerticesMap verticesMap;

    prova(1, 1) = 1;
    prova(1, 2) = 1;
    prova(1, 3) = 1;
    prova(2, 2) = 1;
    prova(2, 3) = 1;
    prova(3, 1) = 1;
    prova(3, 2) = 1;
    prova(3, 3) = 1;

    //Prima componente connessa
    VoronoiVertex v(10, Vector2i(1, 1));
    VoronoiVertex v1(20, Vector2i(1, 2));
    VoronoiVertex v3(30, Vector2i(1, 3));
    VoronoiVertex v4(40, Vector2i(2, 2));
    VoronoiVertex v5(50, Vector2i(2, 3));
    VoronoiVertex v6(60, Vector2i(3, 1));
    VoronoiVertex v7(70, Vector2i(3, 2));
    VoronoiVertex v8(90, Vector2i(3, 3));
    v.setOrder(100);
    v1.setOrder(110);
    v3.setOrder(130);
    v4.setOrder(140);
    v5.setOrder(150);
    v6.setOrder(160);
    v7.setOrder(170);
    v8.setOrder(300);

    verticesMap.insert(VoronoiPair(v7.position(), v7));
    verticesMap.insert(VoronoiPair(v8.position(), v8));
    verticesMap.insert(VoronoiPair(v6.position(), v6));
    verticesMap.insert(VoronoiPair(v5.position(), v5));
    verticesMap.insert(VoronoiPair(v.position(), v));
    verticesMap.insert(VoronoiPair(v1.position(), v1));
    verticesMap.insert(VoronoiPair(v3.position(), v3));
    verticesMap.insert(VoronoiPair(v4.position(), v4));

    MatrixXf mappa2 = MatrixXf::Zero(6, 6);
    mapToMatrix(verticesMap, mappa2);
    savePPM("output1.ppm", mappa2);

    PointersDeque pointersQue;
    int components;
//    breadthFirst(verticesMap, pointersQue, booleanaProva, mappa2, components);
    maxChildrenCounter(verticesMap, mappa2);

    for(VerticesMap::iterator it = verticesMap.begin(); it != verticesMap.end(); ++it)
    {
        VoronoiVertex v = it->second;
        cout << "VERTEX: " << v.position().x() << ", " << v.position().y() << "; #" << v.order() << endl;
        EdgeSet es = v.edgeSet();
        for(EdgeSet::iterator it = es.begin(); it != es.end(); ++it)
        {
            VoronoiEdge e1 = *it;
            cout << "from: " << e1.from()->position().x() << ", " << e1.from()->position().y() << " to: " << e1.to()->position().x() << ", " << e1.to()->position().y() << endl;
        }
    }

    MatrixXf mappa3 = MatrixXf::Zero(6, 6);
    mapToMatrix(verticesMap, mappa3);
    savePPM("output2.ppm", mappa3);

    cout << "NUMBER OF CONNECTED COMPONENTS: " << components << endl;

    exit(0);
}
