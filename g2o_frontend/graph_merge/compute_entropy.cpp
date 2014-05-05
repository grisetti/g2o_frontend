#include <fstream>
#include <iostream>
#include <opencv/cv.h>
#include <opencv/highgui.h>


using namespace std;


void computeEntropy(ifstream& is, double& entropy)
{
    string tag;
    is >> tag;
    cout << tag << endl;
    is >> tag;
    cout << tag << endl;
    int size_x, size_y;
    while(is.peek() == ' ' || is.peek() == '\n')
    {
        is.ignore();
    }
    while(is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> size_x;
    cout << size_x << endl;
    while(is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> size_y;
    cout << size_y << endl;
    while(is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> tag;
    cout << tag << endl;
    float res;
    is >> res;
    cout << res << endl;
    is >> tag;
    cout << tag << endl;
    float ox, oy;
    while(is.peek() == ' ' || is.peek() == '\n')
    {
        is.ignore();
    }
    while(is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> ox;
    cout << ox << endl;
    while(is.peek() == '#')
    {
        is.ignore(255, '\n');
    }
    is >> oy;
    cout << oy << endl;
    is >> tag;
    cout << tag << endl;
    float value = 0;
    for(int y = 0; y < size_y; ++y)
    {
        for(int x = 0; x < size_x; ++x)
        {
            is >> value;
            if(value < 0)
            {
                continue;
            }
            else
            {
                entropy += value;
            }
        }
    }
}


int main(int argc, char** argv)
{
    if(argc < 2 || argc > 3)
    {
        cerr << "usage: " << argv[0] << " <file> " << endl;
        exit(-1);
    }

    ifstream is(argv[1]);
    double entropy = 0;
    computeEntropy(is, entropy);

    cout << "Entropy: " << entropy << endl;

    exit(0);
}
