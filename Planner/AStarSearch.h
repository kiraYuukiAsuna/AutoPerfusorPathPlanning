#pragma once

#include <memory>
#include <tuple>
#include <vector>

#include "Core/AgentNeedle.hpp"
#include "Core/Constraint.hpp"
#include "Core/Geometry.hpp"
#include "Core/Path.hpp"
#include "Core/Position.hpp"
#include "Core/WorldModel.hpp"

// AStar节点结构
struct AStarNode {
	TimePoint timePoint;
	double gCost;  // 从起点到当前节点的实际代价
	double hCost;  // 从当前节点到终点的启发式代价
	double fCost;  // gCost + hCost
	std::shared_ptr<AStarNode> parent;

	AStarNode(const TimePoint& tp, double g = 0, double h = 0, std::shared_ptr<AStarNode> p = nullptr)
		: timePoint(tp), gCost(g), hCost(h), fCost(g + h), parent(p) {}

	bool operator>(const AStarNode& other) const {
		if (std::abs(fCost - other.fCost) < 1e-6) {
			return gCost < other.gCost;	 // 相同f值时优先选择g值大的（更接近目标）
		}
		return fCost > other.fCost;
	}
};

class AStarSearch {
public:
	AStarSearch(const WorldModel& worldModel);

	// 主要搜索接口
	Path findPath(int agentId, const Voxel& start, const Voxel& goal, int startTime = 0,
				  const std::vector<Constraint>& constraints = {});

	// 便利接口：从Position进行搜索
	Path findPath(int agentId, const Position& startPos, const Position& goalPos, int startTime = 0,
				  const std::vector<Constraint>& constraints = {}) {
		Voxel start = m_WorldModel.worldToVoxel(startPos);
		Voxel goal = m_WorldModel.worldToVoxel(goalPos);
		return findPath(agentId, start, goal, startTime, constraints);
	}

	// 设置搜索参数
	void setMaxSearchTime(int maxTime) { m_MaxSearchTime = maxTime; }
	void setMaxSearchNodes(int maxNodes) { m_MaxSearchNodes = maxNodes; }
	void setEnvSamplesK(int k) { m_EnvSamplesK = k; }

	// 启发式函数类型
	enum class HeuristicType {
		Manhattan,	// 曼哈顿距离
		Euclidean,	// 欧几里得距离
		Chebyshev	// 切比雪夫距离（对角线距离）
	};
	void setHeuristicType(HeuristicType type) { m_HeuristicType = type; }

	// 邻居类型
	enum class NeighborType {
		Six,		// 6-连通（上下左右前后）
		TwentySix,	// 26-连通（包括对角线）
		Custom		// 自定义邻居模式
	};
	void setNeighborType(NeighborType type) { m_NeighborType = type; }

	// 自定义邻居偏移（当NeighborType为Custom时使用）
	void setCustomNeighborOffsets(const std::vector<std::tuple<int, int, int>>& offsets) { m_CustomOffsets = offsets; }

	// 获取搜索统计信息
	struct SearchStats {
		int nodesExplored = 0;
		int pathLength = 0;
		double pathCost = 0.0;
		bool pathFound = false;
	};

	const SearchStats& getLastSearchStats() const { return m_LastStats; }

private:
	const WorldModel& m_WorldModel;
	int m_MaxSearchTime;
	int m_MaxSearchNodes;
	HeuristicType m_HeuristicType;
	NeighborType m_NeighborType;
	std::vector<std::tuple<int, int, int>> m_CustomOffsets;
	mutable SearchStats m_LastStats;
	// 连续碰撞采样参数（默认值，可后续暴露 setter）
	int m_EnvSamplesK = 5;	// 每条边对环境的采样次数

	// 辅助函数
	double calculateHeuristic(const Voxel& from, const Voxel& to) const;
	std::vector<TimePoint> getNeighbors(const TimePoint& current) const;
	bool isValidMove(int agentId, const TimePoint& from, const TimePoint& to, const std::vector<Constraint>& constraints) const;
	bool isVoxelObstructed(const Voxel& voxel) const;
	Path reconstructPath(std::shared_ptr<AStarNode> goalNode, int agentId) const;

	// 约束检查
	bool violatesConstraints(int agentId, const TimePoint& from, const TimePoint& to,
							 const std::vector<Constraint>& constraints) const;

	// 移动代价计算
	double getMoveCost(const Voxel& from, const Voxel& to) const;

	// 连续几何检查：针身 vs 环境
	bool bodyCollisionFreeWithEnvironment(int agentId, const TimePoint& from, const TimePoint& to) const;
	// 构建当前时间采样下的胶囊（给定 tipWorld 与方向）
	geom::Capsule makeNeedleCapsule(const AgentNeedle& agent, const Position& tipWorld, const Position& dirUnit) const;
};