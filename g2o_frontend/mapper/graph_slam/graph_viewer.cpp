#include <signal.h>
#include <qapplication.h>
#include <qobject.h>

#include "graph_gui.h"


using namespace std;



int main(int argc, char**argv)
{
    QApplication app(argc, argv);
    GraphGUI* dialog = new GraphGUI();
    dialog->show();

    return app.exec();
}
