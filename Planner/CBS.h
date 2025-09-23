#pragma once

#include <memory>
#include <vector>

#include "AStarSearch.h"
#include "Core/Conflict.hpp"
#include "Core/Constraint.hpp"
#include "Core/Path.hpp"
#include "Core/WorldModel.hpp"
// Geometry and AgentNeedle are used in implementation (CBS.cpp)
// #include "Core/Geometry.hpp"
// #include "Core/AgentNeedle.hpp"

struct CBSNode {
	std::vector<Path> solution;
	std::vector<Constraint> constraints;
	double cost;
	int nodeId;

	CBSNode() : cost(0), nodeId(0) {}

	bool operator>(const CBSNode& other) const {
        return cost > other.cost;
    }

	// 计算解的总代价
	void updateCost() {
		cost = 0;
		for (const auto& path : solution) {
			cost += path.cost;
		}
	}
};

class CBS {
public:
	// 代理信息结构
	struct AgentInfo {
		int agentId;
		Voxel startVoxel;
		Voxel goalVoxel;
		int startTime;

		AgentInfo(int id, const Voxel& start, const Voxel& goal, int time = 0)
			: agentId(id), startVoxel(start), goalVoxel(goal), startTime(time) {}
	};

	CBS(const WorldModel& worldModel);

	// 主要搜索接口
	std::vector<Path> findPaths(const std::vector<AgentInfo>& agents);

	// 便利接口：从Position进行搜索
	std::vector<Path> findPaths(const std::vector<std::tuple<int, Position, Position, int>>& agentPositions);

	// 设置参数
	void setMaxCBSNodes(int maxNodes) { m_MaxCBSNodes = maxNodes; }
	void setMaxSearchTime(int maxTime) { m_MaxSearchTime = maxTime; }
	void setBodySamplesM(int m) { m_BodySamplesM = m; }

	// 获取搜索统计信息
	struct CBSStats {
		int cbsNodesExplored = 0;
		int conflictsResolved = 0;
		bool solutionFound = false;
		double totalCost = 0.0;
		int totalAStarCalls = 0;
	};

	const CBSStats& getLastSearchStats() const { return m_LastStats; }

private:
	const WorldModel& m_WorldModel;
	AStarSearch m_AStar;
	int m_MaxCBSNodes;
	int m_MaxSearchTime;
	int m_NextNodeId;
	mutable CBSStats m_LastStats;
	int m_BodySamplesM = 3;	 // 针身扫掠的时间采样次数

	// 核心CBS算法函数
	std::shared_ptr<CBSNode> createRootNode(const std::vector<AgentInfo>& agents);
	std::vector<Conflict> detectConflicts(const std::vector<Path>& paths);
	Conflict selectConflict(const std::vector<Conflict>& conflicts);
	std::vector<std::shared_ptr<CBSNode>> expandNode(std::shared_ptr<CBSNode> node, const Conflict& conflict,
													 const std::vector<AgentInfo>& agents);

	// 冲突检测辅助函数
	bool hasVertexConflict(const Path& path1, const Path& path2, int time, Voxel& conflictVoxel);
	bool hasEdgeConflict(const Path& path1, const Path& path2, int time, Voxel& voxel1, Voxel& voxel2);
	bool hasBodyConflict(const Path& path1, const Path& path2, int time, Voxel& a_from, Voxel& a_to, Voxel& b_from,
						 Voxel& b_to);

	// 路径规划
	Path planPath(int agentId, const Voxel& start, const Voxel& goal, int startTime,
				  const std::vector<Constraint>& constraints);

	// 验证解的有效性
	bool isValidSolution(const std::vector<Path>& paths);
};

