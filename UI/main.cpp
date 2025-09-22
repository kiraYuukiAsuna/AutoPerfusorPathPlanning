#include <QApplication>
#include "TestMainWindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // Set application metadata
    app.setApplicationName("Path Planning Test UI");
    app.setOrganizationName("PathPlannerTest");
    app.setApplicationDisplayName("Path Planning Algorithm Test Suite");

    // Set application style
    app.setStyle("Fusion");

    // Create and show main window
    TestMainWindow mainWindow;
    mainWindow.show();

    return app.exec();
}