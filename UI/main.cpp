#include <QApplication>
#include <QSurfaceFormat>
#include "TestMainWindow.h"

int main(int argc, char *argv[]) {
    // macOS 下 OpenGL 最高 4.1 Core，设置上下文参数
    QSurfaceFormat fmt;
    fmt.setRenderableType(QSurfaceFormat::OpenGL);
    fmt.setVersion(4, 1);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    fmt.setDepthBufferSize(24);
    fmt.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(fmt);
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