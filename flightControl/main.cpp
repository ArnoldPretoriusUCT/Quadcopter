#include "mainwindow.h"
#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    QThread::currentThread()->setPriority(QThread::HighPriority);
    MainWindow w;
    w.showFullScreen();
    w.show();
    w.windowHandle()->setScreen(qApp->screens()[1]);

    return a.exec();
}
