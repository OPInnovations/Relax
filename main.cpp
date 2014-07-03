#include <QtGui/QApplication>
#include "relax.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ReLax w;
    w.show();
    
    return a.exec();
}
