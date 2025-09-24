#pragma once

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_1_Core>
#include <QOpenGLShaderProgram>
#include <QMatrix4x4>
#include <QVector3D>
#include <QColor>
#include <vector>

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
		// 额外信息用于绘制针身
		float bodyLength = 0.0f;
		float bodyRadius = 0.0f;	   // 当前仅用于显示提示，不做几何绘制
		float angleHorizontal = 0.0f;  // 备用：当没有移动方向时使用
		float angleVertical = 0.0f;	   // 备用：当没有移动方向时使用
	};

	void setObstacles(const std::vector<QVector3D>& obs);
    void setAgents(const std::vector<QVector3D>& starts,
                   const std::vector<QVector3D>& goals,
                   const std::vector<QColor>& colors);
    void setPaths(const std::vector<AgentPath>& paths);
    void updateAgentAnimIndices(const std::vector<int>& indices);
	void setShowNeedles(bool show) {
		showNeedles_ = show;
		update();
	}

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
		bool depthTest = true;	// 是否开启深度测试（用于让描边始终可见）
	};
    void emitGrid(std::vector<Batch>& batches);
    void emitObstacles(std::vector<Batch>& batches);
    void emitAgents(std::vector<Batch>& batches);
    void emitPaths(std::vector<Batch>& batches);
	void emitNeedles(std::vector<Batch>& batches);
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
	bool showNeedles_ = true;

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
