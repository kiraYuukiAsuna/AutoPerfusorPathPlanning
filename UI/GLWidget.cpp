#include "GLWidget.h"
#include <QMouseEvent>
#include <QtMath>

namespace {
inline QVector3D toScene(float x, float y, int sizeX, int sizeY, float unit=1.0f) {
    return { x*unit - (sizeX*unit)/2.0f, 0.0f, y*unit - (sizeY*unit)/2.0f };
}
}

GLWidget::GLWidget(QWidget* parent) : QOpenGLWidget(parent) {}
GLWidget::~GLWidget() {}

void GLWidget::setWorldSize(int sx, int sy, int sz, int voxel) {
    sizeX_ = sx; sizeY_ = sy; sizeZ_ = sz; voxelSize_ = voxel; update();
}

void GLWidget::setShowFlags(bool grid, bool obstacles, bool agents, bool paths) {
    showGrid_ = grid; showObstacles_ = obstacles; showAgents_ = agents; showPaths_ = paths; update();
}

void GLWidget::setObstacles(const std::vector<QVector3D>& obs) { obstacles_ = obs; update(); }

void GLWidget::setAgents(const std::vector<QVector3D>& starts,
                         const std::vector<QVector3D>& goals,
                         const std::vector<QColor>& colors) {
    agentStarts_ = starts; agentGoals_ = goals; agentColors_ = colors; update();
}

void GLWidget::setPaths(const std::vector<AgentPath>& paths) { paths_ = paths; update(); }

void GLWidget::updateAgentAnimIndices(const std::vector<int>& indices) {
    for (size_t i = 0; i < indices.size() && i < paths_.size(); ++i) paths_[i].animIndex = indices[i];
    update();
}

void GLWidget::resetView() { camDistance_ = 80.0f; yaw_ = -45.0f; pitch_ = -35.0f; update(); }
void GLWidget::zoom(float factor) { camDistance_ = qMax(5.0f, camDistance_ * factor); update(); }

void GLWidget::initializeGL() {
    initializeOpenGLFunctions();
    glEnable(GL_DEPTH_TEST);
    // 关闭背面剔除，避免由于三角形绕序不一致造成从某些角度看不见面片
    // glEnable(GL_CULL_FACE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.1f, 0.1f, 0.12f, 1.0f);

    // Simple color shader
    program_.addShaderFromSourceCode(QOpenGLShader::Vertex,
        "#version 410 core\n"
        "layout(location=0) in vec3 aPos;\n"
        "uniform mat4 uMVP;\n"
        "void main(){ gl_Position = uMVP * vec4(aPos,1.0); }\n"
    );
    program_.addShaderFromSourceCode(QOpenGLShader::Fragment,
        "#version 410 core\n"
        "uniform vec3 uColor;\n"
        "uniform float uAlpha;\n"
        "out vec4 FragColor;\n"
        "void main(){ FragColor = vec4(uColor,uAlpha); }\n"
    );
    program_.link();
    uMvpLoc_ = program_.uniformLocation("uMVP");
    uColorLoc_ = program_.uniformLocation("uColor");
    int uAlphaLoc = program_.uniformLocation("uAlpha");

    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vbo_);
}

void GLWidget::resizeGL(int w, int h) {
    proj_.setToIdentity();
    proj_.perspective(45.0f, (h==0?1.0f:float(w)/float(h)), 0.1f, 2000.0f);
}

void GLWidget::paintGL() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // view matrix from yaw/pitch and distance
    QMatrix4x4 rot; rot.rotate(pitch_, 1,0,0); rot.rotate(yaw_, 0,1,0);
    QVector3D tgt(0,0,0);
    QVector3D eye = rot.map(QVector3D(0,0,camDistance_));
    view_.setToIdentity();
    view_.lookAt(eye, tgt, QVector3D(0,1,0));

    std::vector<Batch> batches;
    if (showGrid_) emitGrid(batches);
    if (showObstacles_) emitObstacles(batches);
    if (showAgents_) emitAgents(batches);
    if (showPaths_) emitPaths(batches);
	if (showNeedles_) emitNeedles(batches);
	drawBatches(batches);
}

void GLWidget::emitGrid(std::vector<Batch>& batches) {
    const int step = 1; // 网格按单元绘制，每个单元一条线
    const float unit = float(qMax(1, voxelSize_));
    Batch b; b.color = QColor(200,200,200); b.alpha = 0.35f; b.primitive = GL_LINES; b.lineWidth = 1.0f;
    for (int x = 0; x <= sizeX_; x += step) {
        QVector3D a = toScene(x, 0, sizeX_, sizeY_, unit);
        QVector3D c = toScene(x, sizeY_, sizeX_, sizeY_, unit);
        b.vertices.push_back(a); b.vertices.push_back(c);
    }
    for (int y = 0; y <= sizeY_; y += step) {
        QVector3D a = toScene(0, y, sizeX_, sizeY_, unit);
        QVector3D c = toScene(sizeX_, y, sizeX_, sizeY_, unit);
        b.vertices.push_back(a); b.vertices.push_back(c);
    }
    batches.push_back(std::move(b));
}

void GLWidget::emitObstacles(std::vector<Batch>& batches) {
    // 障碍物对齐到网格单元中心，并落地（底面 y=0）
    const float unit = float(qMax(1, voxelSize_));
	const float boxSize = unit * 0.8f;	// 缩小尺寸以区分网格

	auto emitBox = [&](const QVector3D& center, float size, const QColor& fill){
        float x=center.x(), y=center.y(), z=center.z(); float hh=size*0.5f;
        QVector3D p[8]={{x-hh,y-hh,z-hh},{x+hh,y-hh,z-hh},{x+hh,y-hh,z+hh},{x-hh,y-hh,z+hh},
                        {x-hh,y+hh,z-hh},{x+hh,y+hh,z-hh},{x+hh,y+hh,z+hh},{x-hh,y+hh,z+hh}};
        int faces[][4]={{0,1,2,3},{4,5,6,7},{1,5,6,2},{0,4,7,3},{3,2,6,7},{0,1,5,4}};
		Batch tri;
		tri.color = QColor(120, 120, 120);
		tri.alpha = 1.0f;
		tri.primitive = GL_TRIANGLES;
		for(auto &f:faces){ tri.vertices.push_back(p[f[0]]); tri.vertices.push_back(p[f[1]]); tri.vertices.push_back(p[f[2]]);
                             tri.vertices.push_back(p[f[0]]); tri.vertices.push_back(p[f[2]]); tri.vertices.push_back(p[f[3]]);}        
        batches.push_back(std::move(tri));
        // 高对比度白色描边
        int edges[][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
		Batch line;
		line.color = QColor(255, 255, 255);
		line.alpha = 1.0f;
		line.primitive = GL_LINES;
		line.lineWidth = 2.0f;
		for (auto &e:edges){ line.vertices.push_back(p[e[0]]); line.vertices.push_back(p[e[1]]);}    
        batches.push_back(std::move(line));
    };

    for (const auto& o : obstacles_) {
        // 输入 o.x/o.z 视为网格坐标，o.y 作为中心高度（世界Z）；若为0则默认落地
        QVector3D base = toScene(o.x() + 0.5f, o.z() + 0.5f, sizeX_, sizeY_, unit);
        float yCenter = (o.y() != 0.0f) ? o.y() : (0.5f * unit);
        emitBox({base.x(), yCenter, base.z()}, boxSize, QColor(90,90,90));
    }
}

void GLWidget::emitAgents(std::vector<Batch>& batches) {
    const float unit = float(qMax(1, voxelSize_));
    for (size_t i=0;i<agentStarts_.size() && i<agentColors_.size();++i){
        const float size = 0.9f * unit;
        auto addCube=[&](const QVector3D& a, float sz, const QColor& fill){
            QVector3D base = toScene(a.x() + 0.5f, a.z() + 0.5f, sizeX_, sizeY_, unit);
            float hh = sz * 0.5f;
            // 使用 a.y() 作为立方体中心的世界高度（场景Y）
            float x = base.x(), z = base.z(), y = a.y();
            QVector3D p[8]={{x-hh,y-hh,z-hh},{x+hh,y-hh,z-hh},{x+hh,y-hh,z+hh},{x-hh,y-hh,z+hh},
                            {x-hh,y+hh,z-hh},{x+hh,y+hh,z-hh},{x+hh,y+hh,z+hh},{x-hh,y+hh,z+hh}};
            int faces[][4]={{0,1,2,3},{4,5,6,7},{1,5,6,2},{0,4,7,3},{3,2,6,7},{0,1,5,4}};
            Batch tri; tri.color = fill; tri.alpha = 1.0f; tri.primitive = GL_TRIANGLES;
            for(auto &f:faces){ tri.vertices.push_back(p[f[0]]); tri.vertices.push_back(p[f[1]]); tri.vertices.push_back(p[f[2]]);
                                 tri.vertices.push_back(p[f[0]]); tri.vertices.push_back(p[f[2]]); tri.vertices.push_back(p[f[3]]);}        
            batches.push_back(std::move(tri));
            // outline darker
            int edges[][2]={{0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}};
            Batch line; line.color = QColor::fromHslF(fill.hslHueF(), fill.hslSaturationF(), qMax(0.0, fill.lightnessF()-0.3)); line.alpha=1.0f; line.primitive=GL_LINES; line.lineWidth=2.0f;
            for (auto &e:edges){ line.vertices.push_back(p[e[0]]); line.vertices.push_back(p[e[1]]);}    
            batches.push_back(std::move(line));
        };
        addCube(agentStarts_[i], size, agentColors_[i]);
        if (i<agentGoals_.size()){
            QColor gc = QColor::fromHsv(agentColors_[i].hsvHue(), agentColors_[i].hsvSaturation(), qMin(255, int(agentColors_[i].value()*1.2)));
            addCube(agentGoals_[i], size*0.6f, gc);
        }
    }
}

void GLWidget::emitPaths(std::vector<Batch>& batches) {
    const float unit = float(qMax(1, voxelSize_));
    for (const auto& p : paths_) {
        Batch b; b.color = p.color; b.alpha = 0.95f; b.primitive = GL_LINE_STRIP; b.lineWidth = 3.0f;
        for (const auto& wp : p.waypoints) {
            QVector3D pos = toScene(wp.x() + 0.5f, wp.z() + 0.5f, sizeX_, sizeY_, unit);
            // 高度直接使用传入的数据（期望为世界Z坐标，单位与unit一致）
            // 若传入为0则仍略微抬升避免与地面Z冲突
            float y = (wp.y() != 0.0f ? wp.y() : 0.02f * unit);
            b.vertices.push_back({pos.x(), y, pos.z()});
        }
        batches.push_back(std::move(b));
    }
}

// 以简易方式绘制针身：
// - 使用与路径相同的坐标契约：waypoint 的 (x,z) 为网格坐标（将经 toScene 映射到 X/Z），y 为世界高度（映射到场景 Y）
// - 针尖取 animIndex 位置；方向优先由下一点（场景坐标）计算，退化时用角度估计
// - 仅以线段表示（长度 bodyLength，单位=世界单位=voxelSize_ 缩放后的场景单位），半径以淡色十字提示
void GLWidget::emitNeedles(std::vector<Batch>& batches) {
	if (paths_.empty()) return;
	const float unit = float(qMax(1, voxelSize_));

	auto toScene3D = [&](const QVector3D& wp) -> QVector3D {
		// 将 (gridX, height, gridY) -> 场景 (X, Y=height, Z)
		QVector3D pos = toScene(wp.x() + 0.5f, wp.z() + 0.5f, sizeX_, sizeY_, unit);
		return {pos.x(), wp.y(), pos.z()};
	};

	auto angDir = [](float h, float v) -> QVector3D {
		// 与 Core/Geometry 中方向约定一致：水平角绕Z，垂直角为仰角
		float ch = std::cos(h), sh = std::sin(h);
		float cv = std::cos(v), sv = std::sin(v);
		return QVector3D(cv * ch, sv, cv * sh);
	};

	for (const auto& p : paths_) {
		if (p.waypoints.empty() || p.bodyLength <= 0.0f) continue;
		int idx = std::clamp(p.animIndex, 0, int(p.waypoints.size()) - 1);

		// 针尖（场景坐标）
		QVector3D tipScene = toScene3D(p.waypoints[idx]);

		// 方向（场景坐标下）
		QVector3D dir;
		if (idx + 1 < int(p.waypoints.size())) {
			QVector3D nextScene = toScene3D(p.waypoints[idx + 1]);
			dir = nextScene - tipScene;
			if (dir.lengthSquared() > 1e-6f) dir.normalize();
		}
		if (dir.lengthSquared() < 1e-6f) {
			dir = angDir(p.angleHorizontal, p.angleVertical);
			if (dir.lengthSquared() > 1e-6f)
				dir.normalize();
			else
				dir = QVector3D(1, 0, 0);
		}

		QVector3D tailScene = tipScene - dir * p.bodyLength;
		float r = qMax(0.0f, p.bodyRadius);

		// 计算AABB
		QVector3D minPt, maxPt;
		minPt.setX(std::min(tailScene.x(), tipScene.x()) - r);
		minPt.setY(std::min(tailScene.y(), tipScene.y()) - r);
		minPt.setZ(std::min(tailScene.z(), tipScene.z()) - r);
		maxPt.setX(std::max(tailScene.x(), tipScene.x()) + r);
		maxPt.setY(std::max(tailScene.y(), tipScene.y()) + r);
		maxPt.setZ(std::max(tailScene.z(), tipScene.z()) + r);

		// 8顶点
		QVector3D p8[8];
		p8[0] = {minPt.x(), minPt.y(), minPt.z()};
		p8[1] = {maxPt.x(), minPt.y(), minPt.z()};
		p8[2] = {maxPt.x(), maxPt.y(), minPt.z()};
		p8[3] = {minPt.x(), maxPt.y(), minPt.z()};
		p8[4] = {minPt.x(), minPt.y(), maxPt.z()};
		p8[5] = {maxPt.x(), minPt.y(), maxPt.z()};
		p8[6] = {maxPt.x(), maxPt.y(), maxPt.z()};
		p8[7] = {minPt.x(), maxPt.y(), maxPt.z()};
		int edges[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {4, 5}, {5, 6}, {6, 7}, {7, 4}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
		int faces[6][4] = {{0, 1, 2, 3}, {4, 5, 6, 7}, {1, 5, 6, 2}, {0, 4, 7, 3}, {3, 2, 6, 7}, {0, 1, 5, 4}};

		// 半透明盒体（填充），深度测试开
		Batch boxFill;
		boxFill.primitive = GL_TRIANGLES;
		boxFill.color = QColor(255, 0, 255);
		boxFill.alpha = 0.12f;
		boxFill.depthTest = true;
		for (auto& f : faces) {
			boxFill.vertices.push_back(p8[f[0]]);
			boxFill.vertices.push_back(p8[f[1]]);
			boxFill.vertices.push_back(p8[f[2]]);
			boxFill.vertices.push_back(p8[f[0]]);
			boxFill.vertices.push_back(p8[f[2]]);
			boxFill.vertices.push_back(p8[f[3]]);
		}
		batches.push_back(std::move(boxFill));

		// 高对比度描边（线框），更粗线，关闭深度测试保证始终可见
		Batch boxEdge;
		boxEdge.primitive = GL_LINES;
		boxEdge.color = QColor(255, 255, 255);
		boxEdge.alpha = 0.9f;
		boxEdge.lineWidth = 3.0f;
		boxEdge.depthTest = false;
		for (int i = 0; i < 12; ++i) {
			boxEdge.vertices.push_back(p8[edges[i][0]]);
			boxEdge.vertices.push_back(p8[edges[i][1]]);
		}
		batches.push_back(std::move(boxEdge));
	}
}

void GLWidget::drawBatches(const std::vector<Batch>& batches) {
    if (batches.empty()) return;
    program_.bind();
    QMatrix4x4 mvp = proj_ * view_;
    program_.setUniformValue(uMvpLoc_, mvp);
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vbo_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(QVector3D), reinterpret_cast<void*>(0));

    int uAlphaLoc = program_.uniformLocation("uAlpha");
    for (const auto& b : batches) {
        if (b.vertices.empty()) continue;
		if (b.depthTest)
			glEnable(GL_DEPTH_TEST);
		else
			glDisable(GL_DEPTH_TEST);
		program_.setUniformValue(uColorLoc_, QVector3D(b.color.redF(), b.color.greenF(), b.color.blueF()));
        program_.setUniformValue(uAlphaLoc, b.alpha);
        glBufferData(GL_ARRAY_BUFFER, sizeof(QVector3D)*b.vertices.size(), b.vertices.data(), GL_DYNAMIC_DRAW);
        if (b.primitive == GL_LINES || b.primitive == GL_LINE_STRIP) {
            glLineWidth(b.lineWidth);
        }
        // 为填充面启用多边形偏移，防止与后续线框发生 Z 冲突
        bool usedPolyOffset = false;
        if (b.primitive == GL_TRIANGLES) {
            glEnable(GL_POLYGON_OFFSET_FILL);
            glPolygonOffset(1.1f, 1.0f);
            usedPolyOffset = true;
        }
        glDrawArrays(b.primitive ? b.primitive : GL_LINE_STRIP, 0, GLsizei(b.vertices.size()));
        if (usedPolyOffset) {
            glDisable(GL_POLYGON_OFFSET_FILL);
        }
    }
	glEnable(GL_DEPTH_TEST);
	glDisableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    program_.release();
}

void GLWidget::mousePressEvent(QMouseEvent* e) { lastMousePos_ = e->pos(); }
void GLWidget::mouseMoveEvent(QMouseEvent* e) {
    if (e->buttons() & Qt::LeftButton) {
        QPoint d = e->pos() - lastMousePos_;
        yaw_ += d.x() * 0.3f;
        pitch_ = qBound(-89.0f, pitch_ + d.y()*0.3f, 89.0f);
        update();
    }
    lastMousePos_ = e->pos();
}
void GLWidget::wheelEvent(QWheelEvent* e) {
    float numDegrees = e->angleDelta().y() / 8.0f; // 15° 一刻度
    float numSteps = numDegrees / 15.0f;
    zoom(numSteps < 0 ? 1.15f : 0.87f);
}
