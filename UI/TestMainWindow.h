#pragma once

#include <QMainWindow>
#include <QTimer>
#include <memory>
#include <vector>

#include "../Core/Position.hpp" // for Voxel/TimePoint used in diagnostics

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

// Forward declaration for OpenGL widget
class GLWidget;

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
    // 3D helpers (OpenGL)
    void setup3DScene();
    void updateAgentTransforms3D();
    void logMessage(const QString& message, const QString& type = "info");
    void createTestScenarios();
    void loadScenario(int index);
    void runAStarVisualization();
    void runCBSVisualization();
    void dumpObstaclesAndPaths();
    void animatePath(int agentId);
    void updateStatistics();

    // Test scenarios
    void setupSimpleScenario();
    void setupMazeScenario();
    void setupMultiAgentScenario();
    void setupConflictScenario();
    void setup3DScenario();

    // UI components
    // 3D scene (OpenGL)
    GLWidget* glWidget;
    float cameraDistance = 80.0f; // mapped to GLWidget zoom
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
        // Full-resolution 3D path in voxel-time for accurate diagnostics
        std::vector<TimePoint> voxelPath;
        // Current index along the path for animation (0..path.size()-1)
        int animIndex = 0;
    };

    std::vector<TestAgent> testAgents;
    std::vector<QPointF> obstacles;
    std::vector<std::vector<QPointF>> currentPaths;

    // Cached visuals removed; GLWidget按帧更新

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