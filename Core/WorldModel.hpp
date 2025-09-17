#pragma once

#include "Position.hpp"
#include "AgentNeedle.hpp"
#include "Path.hpp"
#include "Constraint.hpp"
#include <vector>
#include <map>
#include <set>
#include <memory>
#include <mutex>


enum class TaskState { Pending, Assigned, Working, Completed, Failed };

struct Task {
	int taskId;
	Position targetPosition;
	DyeColor requiredColor;
	TaskState state;
	int assignedAgentId;  // -1 if unassigned
	int priority;		  // Higher value means higher priority

	Task(int taskId, Position pos, DyeColor color, int priority)
        : taskId(taskId), targetPosition(pos), requiredColor(color), state(TaskState::Pending),
          assignedAgentId(-1), priority(priority) {}

    void assignToAgent(int agentId) {
        assignedAgentId = agentId;
        state = TaskState::Assigned;
	}
};


struct CameraResource {
public:
	CameraResource() : m_CurrentHolderAgentId(-1) {}
	int getCurrentHolder() const { return m_CurrentHolderAgentId; }
	void setCurrentHolder(int agentId) { m_CurrentHolderAgentId = agentId; }
	bool isAvailable() const { return m_CurrentHolderAgentId == -1; }
	bool release(int agentId) {
		if (m_CurrentHolderAgentId == agentId) {
            m_CurrentHolderAgentId = -1;
            return true;
		}
		return false;
    }
private:
	int m_CurrentHolderAgentId;
};


class WorldModel {
public:
	WorldModel(double gridXSize, double gridYSize, double gridZSize, double voxelSize,
			   const Position& worldOrigin = Position(0, 0, 0))
		: m_GridXSize(gridXSize),
		  m_GridYSize(gridYSize),
		  m_GridZSize(gridZSize),
		  m_VoxelSize(voxelSize),
		  m_WorldOrigin(worldOrigin),
		  nextTaskId(0),
		  m_CurrentTime(0) {}

    void addAgentNeedle(std::shared_ptr<AgentNeedle> agent) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        m_AgentNeedles.push_back(std::move(agent));
	}

	std::shared_ptr<AgentNeedle> getAgentNeedle(int agentId) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        for (const auto& agent : m_AgentNeedles) {
            if (agent->agentId == agentId) {
                return agent;
            }
        }
        return nullptr;
	}

	std::vector<std::shared_ptr<AgentNeedle>> getIdleAgentNeedles() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        std::vector<std::shared_ptr<AgentNeedle>> idleAgents;
        for (const auto& agent : m_AgentNeedles) {
            if (agent->state == NeedleState::Idle) {
                idleAgents.push_back(agent);
            }
        }
		return idleAgents;
	}

	std::vector<std::shared_ptr<AgentNeedle>> getAllAgentNeedles() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        return m_AgentNeedles;
	}

	bool removeAgentNeedle(int agentId) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        auto it = std::find_if(m_AgentNeedles.begin(), m_AgentNeedles.end(),
                                 [agentId](const std::shared_ptr<AgentNeedle>& agent) {
                                     return agent->agentId == agentId;
                                 });
        if (it != m_AgentNeedles.end()) {
			m_AgentNeedles.erase(it);

			// erase path if exists
			m_AgentNeedlePaths.erase(agentId);
			
            return true;
        }
		return false;
    }

	int addTask(const Position& targetPosition, DyeColor requiredColor, int priority=0) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        int taskId = nextTaskId++;
        m_TaskPool.emplace_back(std::make_shared<Task>(taskId, targetPosition, requiredColor, priority));
        return taskId;
	}

	std::shared_ptr<Task> getTask(int taskId) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        for (const auto& task : m_TaskPool) {
            if (task->taskId == taskId) {
                return task;
            }
        }
        return nullptr;
	}

	std::vector<std::shared_ptr<Task>> getPendingTasks() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        std::vector<std::shared_ptr<Task>> pendingTasks;
        for (const auto& task : m_TaskPool) {
            if (task->state == TaskState::Pending) {
                pendingTasks.push_back(task);
            }
        }
        return pendingTasks;
	}

	std::vector<std::shared_ptr<Task>> getAssignedTasks() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        std::vector<std::shared_ptr<Task>> assignedTasks;
        for (const auto& task : m_TaskPool) {
            if (task->state == TaskState::Assigned) {
                assignedTasks.push_back(task);
            }
        }
        return assignedTasks;
	}

	std::vector<std::shared_ptr<Task>> getWorkingTasks() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        std::vector<std::shared_ptr<Task>> workingTasks;
        for (const auto& task : m_TaskPool) {
            if (task->state == TaskState::Working) {
                workingTasks.push_back(task);
            }
        }
        return workingTasks;
    }

	std::vector<std::shared_ptr<Task>> getCompletedTasks() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        std::vector<std::shared_ptr<Task>> completedTasks;
        for (const auto& task : m_TaskPool) {
            if (task->state == TaskState::Completed) {
                completedTasks.push_back(task);
            }
        }
        return completedTasks;
	}

	std::vector<std::shared_ptr<Task>> getFailedTasks() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        std::vector<std::shared_ptr<Task>> failedTasks;
        for (const auto& task : m_TaskPool) {
            if (task->state == TaskState::Failed) {
                failedTasks.push_back(task);
            }
        }
        return failedTasks;
    }

	std::vector<std::shared_ptr<Task>> getAllTasks() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        return m_TaskPool;
    }

	bool removeTask(int taskId) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        auto it = std::remove_if(m_TaskPool.begin(), m_TaskPool.end(),
                                 [taskId](const std::shared_ptr<Task>& task) {
                                     return task->taskId == taskId;
                                 });
        if (it != m_TaskPool.end()) {
            m_TaskPool.erase(it, m_TaskPool.end());
            return true;
        }
        return false;
	}

	void updateTaskState(int taskId, TaskState newState) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        auto task = getTask(taskId);
        if (task) {
            task->state = newState;
        }
	}

	void assignTaskToAgent(int taskId, int agentId) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        auto task = getTask(taskId);
        if (task) {
            task->assignToAgent(agentId);
        }
	}

	void setAgentPath(int agentId, const Path& path) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        m_AgentNeedlePaths[agentId] = path;
	}

	Path getAgentPath(int agentId) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        auto it = m_AgentNeedlePaths.find(agentId);
        if (it != m_AgentNeedlePaths.end()) {
            return it->second;
        }
        return Path();
	}

	void clearAgentPath(int agentId) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        m_AgentNeedlePaths.erase(agentId);
	}

    void addStaticObstacle(const Voxel& voxel) {
        std::lock_guard<std::mutex> lock(m_Mutex);
		m_StaticObstacles.insert(voxel);
    }

    void removeStaticObstacle(const Voxel& voxel) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        m_StaticObstacles.erase(voxel);
    }

    bool hasStaticObstacle(const Voxel& voxel) const {
        std::lock_guard<std::mutex> lock(m_Mutex);
        return m_StaticObstacles.find(voxel) != m_StaticObstacles.end();
    }

    void setCurrentTime(int time) {
        std::lock_guard<std::mutex> lock(m_Mutex);
		m_CurrentTime = time;
	}

	int getCurrentTime() const {
        std::lock_guard<std::mutex> lock(m_Mutex);
        return m_CurrentTime;
	}

	void advanceTime(int step) {
        std::lock_guard<std::mutex> lock(m_Mutex);
        m_CurrentTime += step;
	}
	
	// Check if a voxel is within the valid grid bounds
	bool isVoxelValid(const Voxel& voxel) const {
		int maxX = static_cast<int>(std::ceil(m_GridXSize / m_VoxelSize));
		int maxY = static_cast<int>(std::ceil(m_GridYSize / m_VoxelSize));
		int maxZ = static_cast<int>(std::ceil(m_GridZSize / m_VoxelSize));
		
		return voxel.x >= 0 && voxel.x < maxX &&
			   voxel.y >= 0 && voxel.y < maxY &&
			   voxel.z >= 0 && voxel.z < maxZ;
	}

	const Position& getWorldOrigin() const { return m_WorldOrigin; }

	Position getWorldMax() const {
		return Position(
			m_WorldOrigin.x + m_GridXSize,
			m_WorldOrigin.y + m_GridYSize,
			m_WorldOrigin.z + m_GridZSize
		);
	}

	double getGridXSize() const { return m_GridXSize; }
	double getGridYSize() const { return m_GridYSize; }
	double getGridZSize() const { return m_GridZSize; }
	double getVoxelSize() const { return m_VoxelSize; }

	// Position与Voxel转换函数
	// 将世界坐标转换为体素坐标
	Voxel worldToVoxel(const Position& worldPos) const {
		Position localPos = worldPos - m_WorldOrigin;
		return Voxel(
			static_cast<int>(std::floor(localPos.x / m_VoxelSize)),
			static_cast<int>(std::floor(localPos.y / m_VoxelSize)),
			static_cast<int>(std::floor(localPos.z / m_VoxelSize))
		);
	}
	
	// 将体素坐标转换为世界坐标（体素中心点）
	Position voxelToWorld(const Voxel& voxel) const {
		return Position(
			m_WorldOrigin.x + (voxel.x + 0.5) * m_VoxelSize,
			m_WorldOrigin.y + (voxel.y + 0.5) * m_VoxelSize,
			m_WorldOrigin.z + (voxel.z + 0.5) * m_VoxelSize
		);
	}
	
	// 获取体素的边界（世界坐标）
	std::pair<Position, Position> getVoxelBounds(const Voxel& voxel) const {
		Position minCorner(
			m_WorldOrigin.x + voxel.x * m_VoxelSize,
			m_WorldOrigin.y + voxel.y * m_VoxelSize,
			m_WorldOrigin.z + voxel.z * m_VoxelSize
		);
		Position maxCorner(
			m_WorldOrigin.x + (voxel.x + 1) * m_VoxelSize,
			m_WorldOrigin.y + (voxel.y + 1) * m_VoxelSize,
			m_WorldOrigin.z + (voxel.z + 1) * m_VoxelSize
		);
		return std::make_pair(minCorner, maxCorner);
	}
	
	// 检查世界坐标是否在有效范围内
	bool isWorldPositionValid(const Position& worldPos) const {
		Position localPos = worldPos - m_WorldOrigin;
		return localPos.x >= 0 && localPos.x <= m_GridXSize &&
			   localPos.y >= 0 && localPos.y <= m_GridYSize &&
			   localPos.z >= 0 && localPos.z <= m_GridZSize;
	}

	std::vector<DyeColor> getAvailableDyeColors() const {
		std::vector<DyeColor> availableColors;
		for(auto& agent : m_AgentNeedles) {
			// collect colors from agents
			availableColors.push_back(agent->dyeColor);
        }
        return availableColors;
	}

	void resetWorld() {
        std::lock_guard<std::mutex> lock(m_Mutex);
        m_StaticObstacles.clear();
        m_AgentNeedles.clear();
        m_TaskPool.clear();
        m_AgentNeedlePaths.clear();
        nextTaskId = 0;
        m_CurrentTime = 0;
        if (m_CameraResource) {
            m_CameraResource->setCurrentHolder(-1);
        }
	}

	bool isSystemIdle() const {
        std::lock_guard<std::mutex> lock(m_Mutex);
        for (const auto& agent : m_AgentNeedles) {
            if (agent->state != NeedleState::Idle) {
                return false;
            }
        }
        return true;
	}
	
private:
	// Grid dimensions (in world units)
	double m_GridXSize;
	double m_GridYSize;
	double m_GridZSize;

	// Voxel resolution
	double m_VoxelSize;
	
	// World coordinate system origin offset
	Position m_WorldOrigin;

	std::set<Voxel> m_StaticObstacles;

	std::vector<std::shared_ptr<AgentNeedle>> m_AgentNeedles;
	std::unique_ptr<CameraResource> m_CameraResource;

	std::vector<std::shared_ptr<Task>> m_TaskPool;
	int nextTaskId;

	std::map<int, Path> m_AgentNeedlePaths;

	int m_CurrentTime;

	mutable std::mutex m_Mutex;
};
