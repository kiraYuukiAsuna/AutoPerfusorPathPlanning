#include "TestMainWindow.h"
#include "../Core/WorldModel.hpp"
#include "../Core/AgentNeedle.hpp"
#include "../Planner/AStarSearch.h"
#include "../Planner/CBS.h"

// Qt 3D includes
#include <Qt3DExtras/Qt3DWindow>
#include <Qt3DCore/QEntity>
#include <Qt3DRender/QCamera>
#include <Qt3DExtras/QOrbitCameraController>
#include <Qt3DExtras/QPhongMaterial>
#include <Qt3DExtras/QCylinderMesh>
#include <Qt3DExtras/QSphereMesh>
#include <Qt3DExtras/QCuboidMesh>
#include <Qt3DCore/QTransform>
#include <Qt3DRender/QCameraLens>
#include <Qt3DRender/QDirectionalLight>
#include <QVector3D>
#include <QQuaternion>
#include <QTextEdit>
#include <QListWidget>
#include <QPushButton>
#include <QSpinBox>
#include <QComboBox>
#include <QLabel>
#include <QSlider>
#include <QCheckBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QSplitter>
#include <QDockWidget>
#include <QMenuBar>
#include <QToolBar>
#include <QStatusBar>
#include <QMessageBox>
#include <QDateTime>
#include <QRandomGenerator>
#include <QElapsedTimer>
#include <QtGlobal>

TestMainWindow::TestMainWindow(QWidget* parent)
        : QMainWindow(parent),
            // animation
            isAnimating(false), animationStep(0), animationSpeed(100),
            // visualization
            showGrid(true), showPaths(true), showAgents(true), showObstacles(true),
            // Qt3D pointers
            view3D(nullptr), container3D(nullptr), rootEntity(nullptr), gridEntity(nullptr),
            obstaclesEntity(nullptr), agentsEntity(nullptr), pathsEntity(nullptr),
            camera(nullptr), cameraController(nullptr) {

    setWindowTitle("Path Planning Algorithm Test UI");
    resize(1400, 900);

    setupUI();
    createMenuBar();
    createToolBar();
    createStatusBar();
    initializeWorld();
    createTestScenarios();

    animationTimer = new QTimer(this);
    connect(animationTimer, &QTimer::timeout, this, &TestMainWindow::onAnimationStep);

    logMessage("Test UI initialized successfully", "success");
}

TestMainWindow::~TestMainWindow() {
}

void TestMainWindow::setupUI() {
    // Central widget with 3D view
    QWidget* centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    QVBoxLayout* centralLayout = new QVBoxLayout(centralWidget);
    centralLayout->setContentsMargins(0, 0, 0, 0);

    setup3DScene();
    container3D = QWidget::createWindowContainer(view3D);
    container3D->setFocusPolicy(Qt::StrongFocus);
    centralLayout->addWidget(container3D);

    // Control panel dock
    QDockWidget* controlDock = new QDockWidget("Control Panel", this);
    QWidget* controlWidget = new QWidget();
    QVBoxLayout* controlLayout = new QVBoxLayout(controlWidget);

    // Test controls group
    QGroupBox* testGroup = new QGroupBox("Algorithm Testing");
    QVBoxLayout* testLayout = new QVBoxLayout();

    btnRunAStar = new QPushButton("Run A* Test");
    btnRunCBS = new QPushButton("Run CBS Test");
    btnClear = new QPushButton("Clear Scene");
    btnReset = new QPushButton("Reset World");

    connect(btnRunAStar, &QPushButton::clicked, this, &TestMainWindow::onRunAStarTest);
    connect(btnRunCBS, &QPushButton::clicked, this, &TestMainWindow::onRunCBSTest);
    connect(btnClear, &QPushButton::clicked, this, &TestMainWindow::onClearScene);
    connect(btnReset, &QPushButton::clicked, this, &TestMainWindow::onResetWorld);

    testLayout->addWidget(btnRunAStar);
    testLayout->addWidget(btnRunCBS);
    testLayout->addWidget(btnClear);
    testLayout->addWidget(btnReset);
    testGroup->setLayout(testLayout);

    // Scenario selection
    QGroupBox* scenarioGroup = new QGroupBox("Test Scenarios");
    QVBoxLayout* scenarioLayout = new QVBoxLayout();

    comboScenario = new QComboBox();
    comboScenario->addItems({"Simple Path", "Maze Navigation", "Multi-Agent",
                            "Conflict Resolution", "3D Environment"});
    connect(comboScenario, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &TestMainWindow::onScenarioChanged);

    scenarioLayout->addWidget(new QLabel("Select Scenario:"));
    scenarioLayout->addWidget(comboScenario);
    scenarioGroup->setLayout(scenarioLayout);

    // World settings group
    QGroupBox* worldGroup = new QGroupBox("World Settings");
    QGridLayout* worldLayout = new QGridLayout();

    spinWorldX = new QSpinBox();
    spinWorldX->setRange(10, 100);
    spinWorldX->setValue(50);
    spinWorldX->setSuffix(" units");

    spinWorldY = new QSpinBox();
    spinWorldY->setRange(10, 100);
    spinWorldY->setValue(50);
    spinWorldY->setSuffix(" units");

    spinWorldZ = new QSpinBox();
    spinWorldZ->setRange(1, 50);
    spinWorldZ->setValue(10);
    spinWorldZ->setSuffix(" units");

    spinVoxelSize = new QSpinBox();
    spinVoxelSize->setRange(1, 10);
    spinVoxelSize->setValue(2);
    spinVoxelSize->setSuffix(" units");

    connect(spinWorldX, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &TestMainWindow::onWorldSizeChanged);
    connect(spinWorldY, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &TestMainWindow::onWorldSizeChanged);
    connect(spinWorldZ, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &TestMainWindow::onWorldSizeChanged);
    connect(spinVoxelSize, QOverload<int>::of(&QSpinBox::valueChanged),
            this, &TestMainWindow::onVoxelSizeChanged);

    worldLayout->addWidget(new QLabel("World X:"), 0, 0);
    worldLayout->addWidget(spinWorldX, 0, 1);
    worldLayout->addWidget(new QLabel("World Y:"), 1, 0);
    worldLayout->addWidget(spinWorldY, 1, 1);
    worldLayout->addWidget(new QLabel("World Z:"), 2, 0);
    worldLayout->addWidget(spinWorldZ, 2, 1);
    worldLayout->addWidget(new QLabel("Voxel Size:"), 3, 0);
    worldLayout->addWidget(spinVoxelSize, 3, 1);
    worldGroup->setLayout(worldLayout);

    // Algorithm settings group
    QGroupBox* algoGroup = new QGroupBox("Algorithm Settings");
    QVBoxLayout* algoLayout = new QVBoxLayout();

    comboHeuristic = new QComboBox();
    comboHeuristic->addItems({"Manhattan", "Euclidean", "Chebyshev"});

    comboNeighborType = new QComboBox();
    comboNeighborType->addItems({"6-Connected", "26-Connected"});

    algoLayout->addWidget(new QLabel("Heuristic:"));
    algoLayout->addWidget(comboHeuristic);
    algoLayout->addWidget(new QLabel("Neighbor Type:"));
    algoLayout->addWidget(comboNeighborType);
    algoGroup->setLayout(algoLayout);

    // Agent management group
    QGroupBox* agentGroup = new QGroupBox("Agent Management");
    QVBoxLayout* agentLayout = new QVBoxLayout();

    spinAgentCount = new QSpinBox();
    spinAgentCount->setRange(1, 20);
    spinAgentCount->setValue(3);

    btnAddAgent = new QPushButton("Add Agent");
    btnRemoveAgent = new QPushButton("Remove Agent");

    connect(btnAddAgent, &QPushButton::clicked, this, &TestMainWindow::onAddAgent);
    connect(btnRemoveAgent, &QPushButton::clicked, this, &TestMainWindow::onRemoveAgent);

    agentLayout->addWidget(new QLabel("Agent Count:"));
    agentLayout->addWidget(spinAgentCount);
    agentLayout->addWidget(btnAddAgent);
    agentLayout->addWidget(btnRemoveAgent);
    agentGroup->setLayout(agentLayout);

    // Obstacle management group
    QGroupBox* obstacleGroup = new QGroupBox("Obstacle Management");
    QVBoxLayout* obstacleLayout = new QVBoxLayout();

    spinObstacleCount = new QSpinBox();
    spinObstacleCount->setRange(0, 100);
    spinObstacleCount->setValue(10);

    btnAddObstacle = new QPushButton("Add Random Obstacles");
    connect(btnAddObstacle, &QPushButton::clicked, this, &TestMainWindow::onAddObstacle);

    obstacleLayout->addWidget(new QLabel("Obstacle Count:"));
    obstacleLayout->addWidget(spinObstacleCount);
    obstacleLayout->addWidget(btnAddObstacle);
    obstacleGroup->setLayout(obstacleLayout);

    // Visualization settings
    QGroupBox* visGroup = new QGroupBox("Visualization");
    QVBoxLayout* visLayout = new QVBoxLayout();

    checkShowGrid = new QCheckBox("Show Grid");
    checkShowGrid->setChecked(true);
    checkShowPaths = new QCheckBox("Show Paths");
    checkShowPaths->setChecked(true);
    checkShowAgents = new QCheckBox("Show Agents");
    checkShowAgents->setChecked(true);
    checkShowObstacles = new QCheckBox("Show Obstacles");
    checkShowObstacles->setChecked(true);

    connect(checkShowGrid, &QCheckBox::toggled, this, &TestMainWindow::onShowGridToggled);
    connect(checkShowPaths, &QCheckBox::toggled, this, &TestMainWindow::onShowPathsToggled);
    connect(checkShowAgents, &QCheckBox::toggled, this, &TestMainWindow::onShowAgentsToggled);
    connect(checkShowObstacles, &QCheckBox::toggled, this, &TestMainWindow::onShowObstaclesToggled);

    visLayout->addWidget(checkShowGrid);
    visLayout->addWidget(checkShowPaths);
    visLayout->addWidget(checkShowAgents);
    visLayout->addWidget(checkShowObstacles);
    visGroup->setLayout(visLayout);

    // Animation controls
    QGroupBox* animGroup = new QGroupBox("Animation");
    QVBoxLayout* animLayout = new QVBoxLayout();

    sliderSpeed = new QSlider(Qt::Horizontal);
    sliderSpeed->setRange(10, 500);
    sliderSpeed->setValue(100);

    btnToggleAnimation = new QPushButton("Start Animation");

    connect(sliderSpeed, &QSlider::valueChanged, this, &TestMainWindow::onSpeedChanged);
    connect(btnToggleAnimation, &QPushButton::clicked, this, &TestMainWindow::onToggleAnimation);

    animLayout->addWidget(new QLabel("Animation Speed:"));
    animLayout->addWidget(sliderSpeed);
    animLayout->addWidget(btnToggleAnimation);
    animGroup->setLayout(animLayout);

    // Add all groups to control panel
    controlLayout->addWidget(testGroup);
    controlLayout->addWidget(scenarioGroup);
    controlLayout->addWidget(worldGroup);
    controlLayout->addWidget(algoGroup);
    controlLayout->addWidget(agentGroup);
    controlLayout->addWidget(obstacleGroup);
    controlLayout->addWidget(visGroup);
    controlLayout->addWidget(animGroup);
    controlLayout->addStretch();

    controlDock->setWidget(controlWidget);
    addDockWidget(Qt::LeftDockWidgetArea, controlDock);

    // Log dock
    QDockWidget* logDock = new QDockWidget("System Log", this);
    logWidget = new QTextEdit();
    logWidget->setReadOnly(true);
    logWidget->setMaximumHeight(150);
    logDock->setWidget(logWidget);
    addDockWidget(Qt::BottomDockWidgetArea, logDock);

    // Agent list dock
    QDockWidget* agentDock = new QDockWidget("Agent List", this);
    agentListWidget = new QListWidget();
    agentDock->setWidget(agentListWidget);
    addDockWidget(Qt::RightDockWidgetArea, agentDock);

    // Path list dock
    QDockWidget* pathDock = new QDockWidget("Path Information", this);
    pathListWidget = new QListWidget();
    pathDock->setWidget(pathListWidget);
    addDockWidget(Qt::RightDockWidgetArea, pathDock);

    // Statistics label
    lblStats = new QLabel("Ready");
}

void TestMainWindow::createMenuBar() {
    QMenu* fileMenu = menuBar()->addMenu("&File");

    QAction* newAction = fileMenu->addAction("&New Test");
    newAction->setShortcut(QKeySequence::New);
    connect(newAction, &QAction::triggered, this, &TestMainWindow::onResetWorld);

    QAction* exitAction = fileMenu->addAction("E&xit");
    exitAction->setShortcut(QKeySequence::Quit);
    connect(exitAction, &QAction::triggered, this, &QWidget::close);

    QMenu* testMenu = menuBar()->addMenu("&Test");

    QAction* astarAction = testMenu->addAction("Run &A* Test");
    connect(astarAction, &QAction::triggered, this, &TestMainWindow::onRunAStarTest);

    QAction* cbsAction = testMenu->addAction("Run &CBS Test");
    connect(cbsAction, &QAction::triggered, this, &TestMainWindow::onRunCBSTest);

    QMenu* viewMenu = menuBar()->addMenu("&View");

    QAction* zoomInAction = viewMenu->addAction("Zoom &In");
    zoomInAction->setShortcut(QKeySequence::ZoomIn);
    connect(zoomInAction, &QAction::triggered, [this]() {
        cameraDistance = qMax(5.0f, cameraDistance * 0.8f);
        if (camera)
            camera->setPosition(QVector3D(0.0f, cameraDistance, cameraDistance));
    });

    QAction* zoomOutAction = viewMenu->addAction("Zoom &Out");
    zoomOutAction->setShortcut(QKeySequence::ZoomOut);
    connect(zoomOutAction, &QAction::triggered, [this]() {
        cameraDistance = cameraDistance * 1.25f;
        if (camera)
            camera->setPosition(QVector3D(0.0f, cameraDistance, cameraDistance));
    });

    QAction* resetViewAction = viewMenu->addAction("&Reset View");
    connect(resetViewAction, &QAction::triggered, [this]() {
        cameraDistance = 80.0f;
        if (camera) {
            camera->setPosition(QVector3D(0.0f, cameraDistance, cameraDistance));
            camera->setViewCenter(QVector3D(0.0f, 0.0f, 0.0f));
        }
    });
}

void TestMainWindow::createToolBar() {
    QToolBar* toolBar = addToolBar("Main");

    toolBar->addAction(QIcon(), "Run A*", this, &TestMainWindow::onRunAStarTest);
    toolBar->addAction(QIcon(), "Run CBS", this, &TestMainWindow::onRunCBSTest);
    toolBar->addSeparator();
    toolBar->addAction(QIcon(), "Clear", this, &TestMainWindow::onClearScene);
    toolBar->addAction(QIcon(), "Reset", this, &TestMainWindow::onResetWorld);
}

void TestMainWindow::createStatusBar() {
    lblStatus = new QLabel("Ready");
    lblTime = new QLabel("Time: 0");

    statusBar()->addWidget(lblStatus);
    statusBar()->addWidget(lblTime);
    statusBar()->addPermanentWidget(lblStats);
}

void TestMainWindow::initializeWorld() {
    worldModel = std::make_unique<WorldModel>(
        spinWorldX->value(),
        spinWorldY->value(),
        spinWorldZ->value(),
        spinVoxelSize->value()
    );

    astarSearch = std::make_unique<AStarSearch>(*worldModel);
    cbsSearch = std::make_unique<CBS>(*worldModel);

    updateVisualization();
}

void TestMainWindow::createTestScenarios() {
    // Scenarios will be loaded on demand
}

void TestMainWindow::onRunAStarTest() {
    logMessage("Starting A* pathfinding test...", "info");

    if (testAgents.empty()) {
        logMessage("No agents to test. Please add agents first.", "warning");
        return;
    }

    QElapsedTimer timer;
    timer.start();

    // Run A* for first agent
    auto& agent = testAgents[0];
    Voxel start = worldModel->worldToVoxel(Position(agent.start.x(), agent.start.y(), 0));
    Voxel goal = worldModel->worldToVoxel(Position(agent.goal.x(), agent.goal.y(), 0));

    astarSearch->setHeuristicType(static_cast<AStarSearch::HeuristicType>(comboHeuristic->currentIndex()));
    astarSearch->setNeighborType(comboNeighborType->currentIndex() == 0 ?
                                  AStarSearch::NeighborType::Six :
                                  AStarSearch::NeighborType::TwentySix);

    Path path = astarSearch->findPath(agent.id, start, goal, 0);

    double elapsed = timer.elapsed() / 1000.0;

    if (!path.wayPoints.empty()) {
        // Convert path to visualization format
        agent.path.clear();
        for (const auto& tp : path.wayPoints) {
            Position pos = worldModel->voxelToWorld(tp.voxel);
            agent.path.push_back(QPointF(pos.x, pos.y));
        }
        agent.animIndex = 0; // reset animation to start

        auto stats = astarSearch->getLastSearchStats();
        lastStats.expandedNodes = stats.nodesExplored;
        lastStats.pathLength = stats.pathCost;
        lastStats.computationTime = elapsed;
        lastStats.success = stats.pathFound;

        logMessage(QString("A* search completed successfully in %1 seconds").arg(elapsed), "success");
    logMessage(QString("Path length: %1, Nodes explored: %2").arg(stats.pathLength).arg(stats.nodesExplored), "info");

        updateVisualization();
        updateStatistics();
    } else {
        logMessage("A* search failed to find a path", "error");
        lastStats.success = false;
    }
}

void TestMainWindow::onRunCBSTest() {
    logMessage("Starting CBS multi-agent planning test...", "info");

    if (testAgents.size() < 2) {
        logMessage("CBS requires at least 2 agents. Please add more agents.", "warning");
        return;
    }

    QElapsedTimer timer;
    timer.start();

    // Prepare agent info for CBS
    std::vector<CBS::AgentInfo> agentInfos;
    for (const auto& agent : testAgents) {
        Voxel start = worldModel->worldToVoxel(Position(agent.start.x(), agent.start.y(), 0));
        Voxel goal = worldModel->worldToVoxel(Position(agent.goal.x(), agent.goal.y(), 0));
        agentInfos.emplace_back(agent.id, start, goal, 0);
    }

    // Run CBS
    std::vector<Path> paths = cbsSearch->findPaths(agentInfos);

    double elapsed = timer.elapsed() / 1000.0;

    if (!paths.empty()) {
        // Convert paths to visualization format
        for (size_t i = 0; i < paths.size() && i < testAgents.size(); ++i) {
            testAgents[i].path.clear();
            for (const auto& tp : paths[i].wayPoints) {
                Position pos = worldModel->voxelToWorld(tp.voxel);
                testAgents[i].path.push_back(QPointF(pos.x, pos.y));
            }
            testAgents[i].animIndex = 0; // reset animation index for each agent
        }

        auto stats = cbsSearch->getLastSearchStats();
        lastStats.expandedNodes = stats.cbsNodesExplored;
        lastStats.conflicts = stats.conflictsResolved;
        lastStats.computationTime = elapsed;
        lastStats.success = stats.solutionFound;

        logMessage(QString("CBS planning completed successfully in %1 seconds").arg(elapsed), "success");
        logMessage(QString("CBS nodes: %1, Conflicts resolved: %2").arg(stats.cbsNodesExplored).arg(stats.conflictsResolved), "info");

        updateVisualization();
        updateStatistics();

        // Update path list
        pathListWidget->clear();
        for (size_t i = 0; i < paths.size(); ++i) {
            pathListWidget->addItem(QString("Agent %1: %2 waypoints, cost: %3")
                .arg(i).arg(paths[i].wayPoints.size()).arg(paths[i].cost));
        }
    } else {
        logMessage("CBS failed to find collision-free paths", "error");
        lastStats.success = false;
    }
}

void TestMainWindow::onClearScene() {
    clearGroup(obstaclesEntity);
    clearAgents3D();
    clearPaths3D();
    testAgents.clear();
    obstacles.clear();
    currentPaths.clear();
    agentListWidget->clear();
    pathListWidget->clear();
    updateVisualization();
    logMessage("Scene cleared", "info");
}

void TestMainWindow::onResetWorld() {
    worldModel->resetWorld();
    onClearScene();
    initializeWorld();
    logMessage("World reset", "info");
}

void TestMainWindow::onAddObstacle() {
    int count = spinObstacleCount->value();
    QRandomGenerator* rng = QRandomGenerator::global();

    for (int i = 0; i < count; ++i) {
        double x = rng->bounded(spinWorldX->value());
        double y = rng->bounded(spinWorldY->value());
        obstacles.push_back(QPointF(x, y));

        Voxel voxel = worldModel->worldToVoxel(Position(x, y, 0));
        worldModel->addStaticObstacle(voxel);
    }

    updateVisualization();
    logMessage(QString("Added %1 random obstacles").arg(count), "info");
}

void TestMainWindow::onAddAgent() {
    QRandomGenerator* rng = QRandomGenerator::global();

    TestAgent agent;
    agent.id = testAgents.size();
    agent.start = QPointF(rng->bounded(spinWorldX->value()), rng->bounded(spinWorldY->value()));
    agent.goal = QPointF(rng->bounded(spinWorldX->value()), rng->bounded(spinWorldY->value()));
    agent.color = QColor::fromHsv(rng->bounded(360), 200, 200);

    testAgents.push_back(agent);

    // Add to world model
    auto agentNeedle = std::make_shared<AgentNeedle>(
        agent.id,                                           // agentId
        0.0,                                               // angleHorizontal (default to 0.0)
        0.0,                                               // angleVertical (default to 0.0)
        Voxel(agent.start.x(), agent.start.y(), 0),        // tipPosition (convert to Voxel)
        1.0,                                               // bodyRadius (default to 1.0)
        5.0,                                               // bodyLength (default to 5.0)
        DyeColor::VColor1,                                 // dyeColor
        NeedleState::Idle,                                 // state
        -1                                                 // targetTaskId (default to -1)
    );
    worldModel->addAgentNeedle(agentNeedle);

    // Update UI
    agentListWidget->addItem(QString("Agent %1: (%2,%3) -> (%4,%5)")
        .arg(agent.id)
        .arg(agent.start.x(), 0, 'f', 1)
        .arg(agent.start.y(), 0, 'f', 1)
        .arg(agent.goal.x(), 0, 'f', 1)
        .arg(agent.goal.y(), 0, 'f', 1));

    updateVisualization();
    logMessage(QString("Added agent %1").arg(agent.id), "info");
}

void TestMainWindow::onRemoveAgent() {
    if (!testAgents.empty()) {
        int id = testAgents.back().id;
        testAgents.pop_back();
        worldModel->removeAgentNeedle(id);

        if (agentListWidget->count() > 0) {
            delete agentListWidget->takeItem(agentListWidget->count() - 1);
        }

        updateVisualization();
        logMessage(QString("Removed agent %1").arg(id), "info");
    }
}

void TestMainWindow::onWorldSizeChanged() {
    initializeWorld();
    logMessage("World size updated", "info");
}

void TestMainWindow::onVoxelSizeChanged() {
    initializeWorld();
    logMessage("Voxel size updated", "info");
}

void TestMainWindow::onScenarioChanged(int index) {
    loadScenario(index);
}

void TestMainWindow::loadScenario(int index) {
    onClearScene();

    switch (index) {
        case 0: setupSimpleScenario(); break;
        case 1: setupMazeScenario(); break;
        case 2: setupMultiAgentScenario(); break;
        case 3: setupConflictScenario(); break;
        case 4: setup3DScenario(); break;
    }

    updateVisualization();
}

void TestMainWindow::setupSimpleScenario() {
    logMessage("Loading simple path scenario", "info");

    // Single agent, few obstacles
    TestAgent agent;
    agent.id = 0;
    agent.start = QPointF(5, 5);
    agent.goal = QPointF(45, 45);
    agent.color = Qt::blue;
    testAgents.push_back(agent);

    // Add some obstacles
    for (int i = 15; i < 35; i += 5) {
        obstacles.push_back(QPointF(i, 25));
        Voxel voxel = worldModel->worldToVoxel(Position(i, 25, 0));
        worldModel->addStaticObstacle(voxel);
    }

    agentListWidget->addItem("Agent 0: (5,5) -> (45,45)");
}

void TestMainWindow::setupMazeScenario() {
    logMessage("Loading maze navigation scenario", "info");

    // Create maze-like obstacles
    for (int x = 10; x < 40; x += 2) {
        for (int y = 10; y < 40; y += 10) {
            if ((x/2) % 2 == (y/10) % 2) {
                obstacles.push_back(QPointF(x, y));
                Voxel voxel = worldModel->worldToVoxel(Position(x, y, 0));
                worldModel->addStaticObstacle(voxel);
            }
        }
    }

    // Add agent
    TestAgent agent;
    agent.id = 0;
    agent.start = QPointF(5, 5);
    agent.goal = QPointF(45, 45);
    agent.color = Qt::green;
    testAgents.push_back(agent);

    agentListWidget->addItem("Agent 0: Navigate maze");
}

void TestMainWindow::setupMultiAgentScenario() {
    logMessage("Loading multi-agent scenario", "info");

    // Add multiple agents
    QVector<QPair<QPointF, QPointF>> agentConfigs = {
        {QPointF(5, 5), QPointF(45, 45)},
        {QPointF(45, 5), QPointF(5, 45)},
        {QPointF(5, 45), QPointF(45, 5)},
        {QPointF(45, 45), QPointF(5, 5)}
    };

    for (int i = 0; i < agentConfigs.size(); ++i) {
        TestAgent agent;
        agent.id = i;
        agent.start = agentConfigs[i].first;
        agent.goal = agentConfigs[i].second;
        agent.color = QColor::fromHsv((i * 90) % 360, 200, 200);
        testAgents.push_back(agent);

        agentListWidget->addItem(QString("Agent %1: (%2,%3) -> (%4,%5)")
            .arg(i)
            .arg(agent.start.x())
            .arg(agent.start.y())
            .arg(agent.goal.x())
            .arg(agent.goal.y()));
    }

    // Add central obstacles
    for (int i = 20; i < 30; ++i) {
        obstacles.push_back(QPointF(25, i));
        Voxel voxel = worldModel->worldToVoxel(Position(25, i, 0));
        worldModel->addStaticObstacle(voxel);
    }
}

void TestMainWindow::setupConflictScenario() {
    logMessage("Loading conflict resolution scenario", "info");

    // Two agents with crossing paths
    TestAgent agent1;
    agent1.id = 0;
    agent1.start = QPointF(10, 25);
    agent1.goal = QPointF(40, 25);
    agent1.color = Qt::red;

    TestAgent agent2;
    agent2.id = 1;
    agent2.start = QPointF(25, 10);
    agent2.goal = QPointF(25, 40);
    agent2.color = Qt::blue;

    testAgents.push_back(agent1);
    testAgents.push_back(agent2);

    agentListWidget->addItem("Agent 0: Horizontal path");
    agentListWidget->addItem("Agent 1: Vertical path (conflict at center)");
}

void TestMainWindow::setup3DScenario() {
    logMessage("Loading 3D environment scenario", "info");
    // This would require 3D visualization, simplified for 2D
    setupSimpleScenario();
}

void TestMainWindow::onAnimationStep() {
    if (!isAnimating || testAgents.empty()) return;

    animationStep++;

    // Update agent positions along their paths
    for (auto& agent : testAgents) {
        if (agent.path.size() > 1) {
            // advance along path with wrap-around
            agent.animIndex = (agent.animIndex + 1) % static_cast<int>(agent.path.size());
        }
    }
    // Only update transforms for performance and to avoid flicker
    updateAgentTransforms3D();
}

void TestMainWindow::onSpeedChanged(int value) {
    animationSpeed = value;
    if (isAnimating) {
        animationTimer->setInterval(1000 / animationSpeed);
    }
}

void TestMainWindow::onToggleAnimation() {
    isAnimating = !isAnimating;

    if (isAnimating) {
        // If any agent has a path but animIndex is out of range, reset indices
        for (auto& a : testAgents) {
            if (!a.path.empty() && (a.animIndex < 0 || a.animIndex >= static_cast<int>(a.path.size()))) {
                a.animIndex = 0;
            }
        }
        animationTimer->start(1000 / animationSpeed);
        btnToggleAnimation->setText("Stop Animation");
        logMessage("Animation started", "info");
    } else {
        animationTimer->stop();
        btnToggleAnimation->setText("Start Animation");
        logMessage("Animation stopped", "info");
    }
}

void TestMainWindow::onShowGridToggled(bool checked) {
    showGrid = checked;
    updateVisualization();
}

void TestMainWindow::onShowPathsToggled(bool checked) {
    showPaths = checked;
    updateVisualization();
}

void TestMainWindow::onShowAgentsToggled(bool checked) {
    showAgents = checked;
    updateVisualization();
}

void TestMainWindow::onShowObstaclesToggled(bool checked) {
    showObstacles = checked;
    updateVisualization();
}

void TestMainWindow::updateVisualization() {
    clearGroup(gridEntity);
    clearGroup(obstaclesEntity);
    clearPaths3D();
    clearAgents3D();

    if (showGrid) drawGrid3D();
    if (showObstacles) drawObstacles3D();
    if (showAgents) drawAgents3D();
    if (showPaths) drawPaths3D();
}

void TestMainWindow::setup3DScene() {
    if (view3D) return;
    view3D = new Qt3DExtras::Qt3DWindow();
    rootEntity = new Qt3DCore::QEntity();
    view3D->setRootEntity(rootEntity);

    // Camera
    camera = view3D->camera();
    camera->lens()->setPerspectiveProjection(45.0f, 16.0f/9.0f, 0.1f, 1000.0f);
    camera->setPosition(QVector3D(0.0f, cameraDistance, cameraDistance));
    camera->setViewCenter(QVector3D(0.0f, 0.0f, 0.0f));

    // Orbit controller
    cameraController = new Qt3DExtras::QOrbitCameraController(rootEntity);
    cameraController->setLinearSpeed(50.0f);
    cameraController->setLookSpeed(180.0f);
    cameraController->setCamera(camera);

    // Basic directional light
    auto* lightEntity = new Qt3DCore::QEntity(rootEntity);
    auto* light = new Qt3DRender::QDirectionalLight(lightEntity);
    light->setColor(QColor(255, 255, 255));
    light->setIntensity(0.8f);
    light->setWorldDirection(QVector3D(-1.0f, -1.0f, -0.5f));
    lightEntity->addComponent(light);

    ensureSceneGroups();
}

void TestMainWindow::ensureSceneGroups() {
    if (!gridEntity) gridEntity = new Qt3DCore::QEntity(rootEntity);
    if (!obstaclesEntity) obstaclesEntity = new Qt3DCore::QEntity(rootEntity);
    if (!agentsEntity) agentsEntity = new Qt3DCore::QEntity(rootEntity);
    if (!pathsEntity) pathsEntity = new Qt3DCore::QEntity(rootEntity);
}

void TestMainWindow::clearGroup(Qt3DCore::QEntity* group) {
    if (!group) return;
    const auto children = group->findChildren<Qt3DCore::QEntity*>(QString(), Qt::FindDirectChildrenOnly);
    for (auto* child : children) {
        child->setParent(static_cast<Qt3DCore::QNode*>(nullptr));
        delete child;
    }
}

static inline QVector3D toScenePos(float worldX, float worldY, int worldWidth, int worldHeight, float unit=1.0f) {
    return QVector3D(worldX*unit - (worldWidth*unit)/2.0f, 0.0f, worldY*unit - (worldHeight*unit)/2.0f);
}

void TestMainWindow::drawGrid3D() {
    ensureSceneGroups();
    const int sizeX = spinWorldX->value();
    const int sizeZ = spinWorldY->value();
    const int step = qMax(1, spinVoxelSize->value());
    const float unit = 1.0f;

    auto material = new Qt3DExtras::QPhongMaterial(gridEntity);
    material->setAmbient(QColor(200,200,200));

    // Lines parallel to Z (varying X)
    for (int x = 0; x <= sizeX; x += step) {
        auto ent = new Qt3DCore::QEntity(gridEntity);
        auto mesh = new Qt3DExtras::QCylinderMesh();
        mesh->setRadius(0.02f);
        mesh->setLength(sizeZ * unit);
        auto tr = new Qt3DCore::QTransform();
        tr->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(1,0,0), 90));
        tr->setTranslation(toScenePos(x, sizeZ/2.0f, sizeX, sizeZ, unit));
        ent->addComponent(mesh);
        ent->addComponent(material);
        ent->addComponent(tr);
    }

    // Lines parallel to X (varying Z)
    for (int z = 0; z <= sizeZ; z += step) {
        auto ent = new Qt3DCore::QEntity(gridEntity);
        auto mesh = new Qt3DExtras::QCylinderMesh();
        mesh->setRadius(0.02f);
        mesh->setLength(sizeX * unit);
        auto tr = new Qt3DCore::QTransform();
        tr->setRotation(QQuaternion::fromAxisAndAngle(QVector3D(0,0,1), 90));
        tr->setTranslation(toScenePos(sizeX/2.0f, z, sizeX, sizeZ, unit));
        ent->addComponent(mesh);
        ent->addComponent(material);
        ent->addComponent(tr);
    }
}

void TestMainWindow::drawObstacles3D() {
    ensureSceneGroups();
    const float unit = 1.0f;
    int size = qMax(1, spinVoxelSize->value());
    for (const auto& obs : obstacles) {
        auto ent = new Qt3DCore::QEntity(obstaclesEntity);
        auto mesh = new Qt3DExtras::QCuboidMesh();
        mesh->setXExtent(size*unit);
        mesh->setYExtent(size*unit*0.5f);
        mesh->setZExtent(size*unit);
        auto mat = new Qt3DExtras::QPhongMaterial(ent);
        mat->setDiffuse(QColor(90,90,90));
        auto tr = new Qt3DCore::QTransform();
        QVector3D pos = toScenePos(obs.x(), obs.y(), spinWorldX->value(), spinWorldY->value(), unit);
        tr->setTranslation(QVector3D(pos.x(), mesh->yExtent()/2.0f, pos.z()));
        ent->addComponent(mesh);
        ent->addComponent(mat);
        ent->addComponent(tr);
    }
}

void TestMainWindow::drawAgents3D() {
    ensureSceneGroups();
    const float unit = 1.0f;
    float radius = 0.4f * qMax(1, spinVoxelSize->value());
    for (const auto& agent : testAgents) {
        // Determine current position: if animating and have path, use current waypoint; else use start
        QPointF current = agent.start;
        if (!agent.path.empty()) {
            int idx = qBound(0, agent.animIndex, static_cast<int>(agent.path.size()) - 1);
            current = agent.path[idx];
        }
        auto ent = new Qt3DCore::QEntity(agentsEntity);
        auto mesh = new Qt3DExtras::QSphereMesh();
        mesh->setRadius(radius);
        auto mat = new Qt3DExtras::QPhongMaterial(ent);
        mat->setDiffuse(agent.color);
        auto tr = new Qt3DCore::QTransform();
        QVector3D pos = toScenePos(current.x(), current.y(), spinWorldX->value(), spinWorldY->value(), unit);
        tr->setTranslation(QVector3D(pos.x(), radius, pos.z()));
        ent->addComponent(mesh);
        ent->addComponent(mat);
        ent->addComponent(tr);

        AgentVisual visual;
        visual.entity = ent;
        visual.transform = tr;
        visual.material = mat;

        // Goal marker as wireframe-ish cube (solid cube with different color)
        auto goalEnt = new Qt3DCore::QEntity(agentsEntity);
        auto goalMesh = new Qt3DExtras::QCuboidMesh();
        goalMesh->setXExtent(radius*1.2f);
        goalMesh->setYExtent(radius*1.2f);
        goalMesh->setZExtent(radius*1.2f);
        auto goalMat = new Qt3DExtras::QPhongMaterial(goalEnt);
        goalMat->setDiffuse(agent.color.lighter(130));
        auto goalTr = new Qt3DCore::QTransform();
        QVector3D goalPos = toScenePos(agent.goal.x(), agent.goal.y(), spinWorldX->value(), spinWorldY->value(), unit);
        goalTr->setTranslation(QVector3D(goalPos.x(), radius*1.2f/2.0f, goalPos.z()));
        goalEnt->addComponent(goalMesh);
        goalEnt->addComponent(goalMat);
        goalEnt->addComponent(goalTr);

        visual.goalEntity = goalEnt;
        visual.goalTransform = goalTr;
        agentVisuals[agent.id] = visual;
    }
}

void TestMainWindow::drawPaths3D() {
    ensureSceneGroups();
    const float unit = 1.0f;
    float thickness = 0.08f * qMax(1, spinVoxelSize->value());
    for (const auto& agent : testAgents) {
        if (agent.path.size() < 2) continue;
        auto mat = new Qt3DExtras::QPhongMaterial(pathsEntity);
        mat->setDiffuse(agent.color);
        for (size_t i = 1; i < agent.path.size(); ++i) {
            QPointF p0 = agent.path[i-1];
            QPointF p1 = agent.path[i];
            QVector3D a = toScenePos(p0.x(), p0.y(), spinWorldX->value(), spinWorldY->value(), unit);
            QVector3D b = toScenePos(p1.x(), p1.y(), spinWorldX->value(), spinWorldY->value(), unit);
            QVector3D d = b - a;
            float len = d.length();
            if (len <= 1e-4f) continue;
            d.normalize();
            // Cylinder along segment
            auto segEnt = new Qt3DCore::QEntity(pathsEntity);
            auto mesh = new Qt3DExtras::QCylinderMesh();
            mesh->setRadius(thickness);
            mesh->setLength(len);
            auto tr = new Qt3DCore::QTransform();
            // rotate Y-axis (0,1,0) to segment direction
            QQuaternion rot = QQuaternion::rotationTo(QVector3D(0,1,0), d);
            tr->setRotation(rot);
            tr->setTranslation(a + d * (len/2.0f));
            segEnt->addComponent(mesh);
            segEnt->addComponent(mat);
            segEnt->addComponent(tr);
        }
    }
}

void TestMainWindow::updateStatistics() {
    QString statsText = QString("Nodes: %1 | Time: %2s | Success: %3")
        .arg(lastStats.expandedNodes)
        .arg(lastStats.computationTime, 0, 'f', 3)
        .arg(lastStats.success ? "Yes" : "No");

    if (lastStats.conflicts > 0) {
        statsText += QString(" | Conflicts: %1").arg(lastStats.conflicts);
    }

    lblStats->setText(statsText);
}

void TestMainWindow::logMessage(const QString& message, const QString& type) {
    QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    QString formattedMsg = QString("[%1] %2").arg(timestamp).arg(message);

    if (type == "error") {
        formattedMsg = QString("<span style='color: red;'>%1</span>").arg(formattedMsg);
    } else if (type == "warning") {
        formattedMsg = QString("<span style='color: orange;'>%1</span>").arg(formattedMsg);
    } else if (type == "success") {
        formattedMsg = QString("<span style='color: green;'>%1</span>").arg(formattedMsg);
    }

    logWidget->append(formattedMsg);
}

void TestMainWindow::onVisualizationUpdate() {
    updateVisualization();
}

void TestMainWindow::updateAgentTransforms3D() {
    const float unit = 1.0f;
    float radius = 0.4f * qMax(1, spinVoxelSize->value());
    for (const auto& agent : testAgents) {
        auto it = agentVisuals.find(agent.id);
        if (it == agentVisuals.end()) continue;
        QPointF current = agent.start;
        if (!agent.path.empty()) {
            int idx = qBound(0, agent.animIndex, static_cast<int>(agent.path.size()) - 1);
            current = agent.path[idx];
        }
        QVector3D pos = toScenePos(current.x(), current.y(), spinWorldX->value(), spinWorldY->value(), unit);
        if (it->second.transform) {
            it->second.transform->setTranslation(QVector3D(pos.x(), radius, pos.z()));
        }
    }
}

void TestMainWindow::clearAgents3D() {
    for (auto& kv : agentVisuals) {
        if (kv.second.entity) { kv.second.entity->setParent(static_cast<Qt3DCore::QNode*>(nullptr)); delete kv.second.entity; }
        if (kv.second.goalEntity) { kv.second.goalEntity->setParent(static_cast<Qt3DCore::QNode*>(nullptr)); delete kv.second.goalEntity; }
    }
    agentVisuals.clear();
}

void TestMainWindow::clearPaths3D() {
    clearGroup(pathsEntity);
}