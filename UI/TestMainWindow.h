#pragma once

#include <QMainWindow>
#include <QTimer>
#include <memory>
#include <vector>
#include <unordered_map>

QT_BEGIN_NAMESPACE
class QTextEdit;
class QListWidget;
class QPushButton;
class QSpinBox;
class QComboBox;
class QLabel;
class QSlider;
class QCheckBox;
QT_END_NAMESPACE

class WorldModel;
class AStarSearch;
class CBS;

// Forward declarations for Qt 3D
namespace Qt3DCore { class QEntity; }
namespace Qt3DRender { class QCamera; }
namespace Qt3DExtras { class Qt3DWindow; class QOrbitCameraController; class QPhongMaterial; }
namespace Qt3DCore { class QTransform; }

class TestMainWindow : public QMainWindow {
    Q_OBJECT

public:
    TestMainWindow(QWidget* parent = nullptr);
    ~TestMainWindow();

private slots:
    void onRunAStarTest();
    void onRunCBSTest();
    void onClearScene();
    void onResetWorld();
    void onAddObstacle();
    void onAddAgent();
    void onRemoveAgent();
    void onWorldSizeChanged();
    void onVoxelSizeChanged();
    void onVisualizationUpdate();
    void onAnimationStep();
    void onSpeedChanged(int value);
    void onToggleAnimation();
    void onScenarioChanged(int index);
    void onShowGridToggled(bool checked);
    void onShowPathsToggled(bool checked);
    void onShowAgentsToggled(bool checked);
    void onShowObstaclesToggled(bool checked);

private:
    void setupUI();
    void createMenuBar();
    void createToolBar();
    void createStatusBar();
    void initializeWorld();
    void updateVisualization();
    // 3D helpers
    void setup3DScene();
    void ensureSceneGroups();
    void clearGroup(Qt3DCore::QEntity* group);
    void drawGrid3D();
    void drawObstacles3D();
    void drawAgents3D();
    void drawPaths3D();
    void updateAgentTransforms3D();
    void clearAgents3D();
    void clearPaths3D();
    void logMessage(const QString& message, const QString& type = "info");
    void createTestScenarios();
    void loadScenario(int index);
    void runAStarVisualization();
    void runCBSVisualization();
    void animatePath(int agentId);
    void updateStatistics();

    // Test scenarios
    void setupSimpleScenario();
    void setupMazeScenario();
    void setupMultiAgentScenario();
    void setupConflictScenario();
    void setup3DScenario();

    // UI components
    // 3D scene
    Qt3DExtras::Qt3DWindow* view3D;
    QWidget* container3D;
    Qt3DCore::QEntity* rootEntity;
    Qt3DCore::QEntity* gridEntity;
    Qt3DCore::QEntity* obstaclesEntity;
    Qt3DCore::QEntity* agentsEntity;
    Qt3DCore::QEntity* pathsEntity;
    Qt3DRender::QCamera* camera;
    Qt3DExtras::QOrbitCameraController* cameraController;
    float cameraDistance = 80.0f;
    QTextEdit* logWidget;
    QListWidget* agentListWidget;
    QListWidget* pathListWidget;

    // Control panel widgets
    QPushButton* btnRunAStar;
    QPushButton* btnRunCBS;
    QPushButton* btnClear;
    QPushButton* btnReset;
    QPushButton* btnAddObstacle;
    QPushButton* btnAddAgent;
    QPushButton* btnRemoveAgent;
    QPushButton* btnToggleAnimation;

    QSpinBox* spinWorldX;
    QSpinBox* spinWorldY;
    QSpinBox* spinWorldZ;
    QSpinBox* spinVoxelSize;
    QSpinBox* spinAgentCount;
    QSpinBox* spinObstacleCount;

    QComboBox* comboScenario;
    QComboBox* comboHeuristic;
    QComboBox* comboNeighborType;

    QSlider* sliderSpeed;
    QLabel* lblStatus;
    QLabel* lblTime;
    QLabel* lblStats;

    QCheckBox* checkShowGrid;
    QCheckBox* checkShowPaths;
    QCheckBox* checkShowAgents;
    QCheckBox* checkShowObstacles;

    // Core components
    std::unique_ptr<WorldModel> worldModel;
    std::unique_ptr<AStarSearch> astarSearch;
    std::unique_ptr<CBS> cbsSearch;

    // Animation
    QTimer* animationTimer;
    bool isAnimating;
    int animationStep;
    int animationSpeed;

    // Visualization state
    bool showGrid;
    bool showPaths;
    bool showAgents;
    bool showObstacles;

    // Test data
    struct TestAgent {
        int id;
        QPointF start;
        QPointF goal;
        QColor color;
        std::vector<QPointF> path;
        // Current index along the path for animation (0..path.size()-1)
        int animIndex = 0;
    };

    std::vector<TestAgent> testAgents;
    std::vector<QPointF> obstacles;
    std::vector<std::vector<QPointF>> currentPaths;

    // Cached visuals for efficient animation updates
    struct AgentVisual {
        Qt3DCore::QEntity* entity = nullptr;       // sphere
        Qt3DCore::QEntity* goalEntity = nullptr;   // cube for goal
        Qt3DCore::QTransform* transform = nullptr;
        Qt3DCore::QTransform* goalTransform = nullptr;
        Qt3DExtras::QPhongMaterial* material = nullptr;
    };
    std::unordered_map<int, AgentVisual> agentVisuals; // key: agent id

    // Statistics
    struct TestStats {
        int totalNodes;
        int expandedNodes;
        double pathLength;
        double computationTime;
        int conflicts;
        bool success;
    };

    TestStats lastStats;
};