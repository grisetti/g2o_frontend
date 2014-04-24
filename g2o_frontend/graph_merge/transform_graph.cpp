#include <vector>

#include "drawable.h"
#include "graph.h"
#include "graph_matcher.h"
#include "graph_gui.h"
#include "viewer.h"
#include "utility.h"


#define MARKUSED(x) x=x


using namespace std;
using namespace Eigen;


int main(int argc, char** argv)
{
    int c = argc - 1;
    ofstream os("4.g2o");

    Isometry2d rotation = utility::v2t(Vector3d(0, 0, -1.57));
    while(c < argc)
    {
        cerr << "Reading file " << argv[c] << endl;
        ifstream is(argv[c]);

        string tag;
        const int maxDim = 32000;
        while(is)
        {
            char line[maxDim];
            is.getline(line, maxDim);
            //  cerr << "line: " << line << endl;
            istringstream ls(line);
            ls >> tag;
            if(tag == "VERTEX_SE2")
            {
                int id;
                Vector3d v;
                ls >> id >> v.x() >> v.y() >> v.z();
                Vector3d last = utility::t2v(rotation * utility::v2t(v));
                os << tag << " " << id  << " " << last.x() << " " << last.y() << " " << last.z() << endl;
            }
            else
            {
                os << ls.str() << endl;
            }
        }
        c++;
    }
    os.close();
}
