#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_1_Core>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QVector3D>
#include <QColor>
#include <vector>

// macOS 的 OpenGL 驱动最高为 4.1 Core，接口与 4.5 用到的基础功能一致
class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions_4_1_Core {
    Q_OBJECT
public:
    explicit GLWidget(QWidget* parent = nullptr);
    ~GLWidget() override;

    // 数据设置接口
    void setWorldSize(int sizeX, int sizeY, int sizeZ, int voxelSize);
    void setShowFlags(bool grid, bool obstacles, bool agents, bool paths);

    struct AgentPath {
        int id;
        QColor color;
        std::vector<QVector3D> waypoints; // y 作为高度
        int animIndex = 0;
    };

    void setObstacles(const std::vector<QVector3D>& obs);
    void setAgents(const std::vector<QVector3D>& starts,
                   const std::vector<QVector3D>& goals,
                   const std::vector<QColor>& colors);
    void setPaths(const std::vector<AgentPath>& paths);
    void updateAgentAnimIndices(const std::vector<int>& indices);

    // 简单相机控制
    void resetView();
    void zoom(float factor);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

    void mousePressEvent(QMouseEvent* e) override;
    void mouseMoveEvent(QMouseEvent* e) override;
    void wheelEvent(QWheelEvent* e) override;

private:
    // 绘制帮助
    struct Batch {
        QColor color;
        float alpha = 1.0f;
        float lineWidth = 1.0f;
        unsigned int primitive = 0; // GL_LINES, GL_LINE_STRIP, GL_TRIANGLES
        std::vector<QVector3D> vertices;
    };
    void emitGrid(std::vector<Batch>& batches);
    void emitObstacles(std::vector<Batch>& batches);
    void emitAgents(std::vector<Batch>& batches);
    void emitPaths(std::vector<Batch>& batches);
    void drawBatches(const std::vector<Batch>& batches);

    // 矩阵
    QMatrix4x4 proj_;
    QMatrix4x4 view_;

    // 相机参数
    float camDistance_ = 80.0f;
    float yaw_ = -45.0f;   // 绕Y
    float pitch_ = -35.0f; // 绕X
    QPoint lastMousePos_;

    // 世界参数
    int sizeX_ = 50;
    int sizeY_ = 50;
    int sizeZ_ = 10;
    int voxelSize_ = 2;
    bool showGrid_ = true;
    bool showObstacles_ = true;
    bool showAgents_ = true;
    bool showPaths_ = true;

    // 数据
    std::vector<QVector3D> obstacles_;
    std::vector<QVector3D> agentStarts_;
    std::vector<QVector3D> agentGoals_;
    std::vector<QColor> agentColors_;
    std::vector<AgentPath> paths_;

    // GL resources
    QOpenGLShaderProgram program_;
    GLuint vao_ = 0;
    GLuint vbo_ = 0;
    int uMvpLoc_ = -1;
    int uColorLoc_ = -1;
};
