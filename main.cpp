#include <QApplication>
#include "display_window.h"

int main(int argc, char *argv[])
{

    QApplication app(argc, argv);

    DisplayWindow window;
    window.setWindowTitle("Data Structures Tester");
    window.resize(800, 600);
    window.show();

    return app.exec();
}
