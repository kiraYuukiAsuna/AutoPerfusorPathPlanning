#include "CBS.h"

#include <algorithm>
#include <iostream>


CBS::CBS(const WorldModel& worldModel)
	: m_WorldModel(worldModel), m_AStar(worldModel), m_MaxCBSNodes(1000), m_MaxSearchTime(30), m_NextNodeId(0) {}

std::vector<Path> CBS::findPaths(const std::vector<AgentInfo>& agents) {
	// 重置统计信息
	m_LastStats = CBSStats{};
	m_NextNodeId = 0;

	if (agents.empty()) {
		return {};
	}

	// 创建优先队列用于CBS搜索
	std::priority_queue<std::shared_ptr<CBSNode>, std::vector<std::shared_ptr<CBSNode>>,
						std::function<bool(const std::shared_ptr<CBSNode>&, const std::shared_ptr<CBSNode>&)>>
		openList([](const auto& a, const auto& b) { return *a > *b; });

	// 创建根节点
	auto rootNode = createRootNode(agents);
	if (!rootNode) {
		std::cout << "Failed to create root node - no initial paths found" << std::endl;
		return {};
	}

	openList.push(rootNode);

	while (!openList.empty() && m_LastStats.cbsNodesExplored < m_MaxCBSNodes) {
		auto currentNode = openList.top();
		openList.pop();
		m_LastStats.cbsNodesExplored++;

		// 检测冲突
		auto conflicts = detectConflicts(currentNode->solution);

		if (conflicts.empty()) {
			// 找到无冲突解
			m_LastStats.solutionFound = true;
			m_LastStats.totalCost = currentNode->cost;
			std::cout << "CBS found solution! Nodes explored: " << m_LastStats.cbsNodesExplored
					  << ", Total cost: " << currentNode->cost << std::endl;
			return currentNode->solution;
		}

		// 选择一个冲突进行解决
		Conflict conflict = selectConflict(conflicts);
		m_LastStats.conflictsResolved++;

		std::cout << "Resolving conflict between agents " << conflict.agent1 << " and " << conflict.agent2 << " at time "
				  << conflict.time << std::endl;

		// 扩展节点
		auto childNodes = expandNode(currentNode, conflict, agents);

		for (auto& child : childNodes) {
			if (child) {
				openList.push(child);
			}
		}
	}

	std::cout << "CBS failed to find solution. Nodes explored: " << m_LastStats.cbsNodesExplored << std::endl;
	return {};
}

std::vector<Path> CBS::findPaths(const std::vector<std::tuple<int, Position, Position, int>>& agentPositions) {
	std::vector<AgentInfo> agents;
	agents.reserve(agentPositions.size());

	for (const auto& [agentId, startPos, goalPos, startTime] : agentPositions) {
		Voxel startVoxel = m_WorldModel.worldToVoxel(startPos);
		Voxel goalVoxel = m_WorldModel.worldToVoxel(goalPos);
		agents.emplace_back(agentId, startVoxel, goalVoxel, startTime);
	}

	return findPaths(agents);
}

std::shared_ptr<CBSNode> CBS::createRootNode(const std::vector<AgentInfo>& agents) {
	auto rootNode = std::make_shared<CBSNode>();
	rootNode->nodeId = m_NextNodeId++;
	rootNode->solution.reserve(agents.size());

	// 为每个代理单独规划路径（无约束）
	for (const auto& agent : agents) {
		Path path = planPath(agent.agentId, agent.startVoxel, agent.goalVoxel, agent.startTime, {});

		if (path.empty()) {
			std::cout << "No path found for agent " << agent.agentId << std::endl;
			return nullptr;
		}

		rootNode->solution.push_back(path);
	}

	rootNode->updateCost();
	return rootNode;
}

std::vector<Conflict> CBS::detectConflicts(const std::vector<Path>& paths) {
	std::vector<Conflict> conflicts;

	// 检查每对代理之间的冲突
	for (size_t i = 0; i < paths.size(); i++) {
		for (size_t j = i + 1; j < paths.size(); j++) {
			const Path& path1 = paths[i];
			const Path& path2 = paths[j];

			// 确定需要检查的时间范围
			int maxTime = std::max(path1.getEndTime(), path2.getEndTime());

			for (int time = 0; time <= maxTime; time++) {
				// 检查顶点冲突
				Voxel conflictVoxel;
				if (hasVertexConflict(path1, path2, time, conflictVoxel)) {
					conflicts.emplace_back(path1.agentId, path2.agentId, conflictVoxel, time, ConflictType::Vertex);
				}

				// 检查边冲突
				if (time < maxTime) {
					Voxel voxel1, voxel2;
					if (hasEdgeConflict(path1, path2, time, voxel1, voxel2)) {
						conflicts.emplace_back(path1.agentId, path2.agentId, voxel1, voxel2, time, ConflictType::Edge);
					}
				}
			}
		}
	}

	return conflicts;
}

Conflict CBS::selectConflict(const std::vector<Conflict>& conflicts) {
	// 简单策略：选择第一个冲突
	// 可以改进为选择最早时间的冲突或其他启发式策略
	return conflicts[0];
}

std::vector<std::shared_ptr<CBSNode>> CBS::expandNode(std::shared_ptr<CBSNode> node, const Conflict& conflict,
													  const std::vector<AgentInfo>& agents) {
	std::vector<std::shared_ptr<CBSNode>> childNodes;

	// 为冲突中的每个代理创建一个子节点
	std::vector<int> conflictAgents = {conflict.agent1};
	if (conflict.agent2 != -1) {
		conflictAgents.push_back(conflict.agent2);
	}

	for (int agentId : conflictAgents) {
		auto childNode = std::make_shared<CBSNode>();
		childNode->nodeId = m_NextNodeId++;
		childNode->constraints = node->constraints;
		childNode->solution = node->solution;

		// 添加新约束
		if (conflict.type == ConflictType::Vertex) {
			childNode->constraints.emplace_back(agentId, conflict.voxel1, conflict.time, ConstraintType::Vertex);
		} else if (conflict.type == ConflictType::Edge) {
			if (agentId == conflict.agent1) {
				childNode->constraints.emplace_back(agentId, conflict.voxel1, conflict.voxel2, conflict.time, -1,
													ConstraintType::Edge);
			} else {
				childNode->constraints.emplace_back(agentId, conflict.voxel2, conflict.voxel1, conflict.time, -1,
													ConstraintType::Edge);
			}
		}

		// 找到对应的代理信息
		auto agentIt =
			std::find_if(agents.begin(), agents.end(), [agentId](const AgentInfo& agent) { return agent.agentId == agentId; });

		if (agentIt == agents.end()) {
			std::cout << "Agent " << agentId << " not found in agent list" << std::endl;
			continue;
		}

		// 为受约束的代理重新规划路径
		Path newPath = planPath(agentId, agentIt->startVoxel, agentIt->goalVoxel, agentIt->startTime, childNode->constraints);

		if (newPath.empty()) {
			// 无法找到满足约束的路径，跳过这个子节点
			std::cout << "No path found for agent " << agentId << " with new constraints" << std::endl;
			continue;
		}

		// 更新解中的路径
		for (auto& path : childNode->solution) {
			if (path.agentId == agentId) {
				path = newPath;
				break;
			}
		}

		childNode->updateCost();
		childNodes.push_back(childNode);
	}

	return childNodes;
}

bool CBS::hasVertexConflict(const Path& path1, const Path& path2, int time, Voxel& conflictVoxel) {
	auto pos1 = path1.getPositionAtTime(time);
	auto pos2 = path2.getPositionAtTime(time);

	if (pos1.has_value() && pos2.has_value()) {
		if (pos1->voxel == pos2->voxel) {
			conflictVoxel = pos1->voxel;
			return true;
		}
	}

	return false;
}

bool CBS::hasEdgeConflict(const Path& path1, const Path& path2, int time, Voxel& voxel1, Voxel& voxel2) {
	auto pos1_t = path1.getPositionAtTime(time);
	auto pos1_t1 = path1.getPositionAtTime(time + 1);
	auto pos2_t = path2.getPositionAtTime(time);
	auto pos2_t1 = path2.getPositionAtTime(time + 1);

	if (pos1_t.has_value() && pos1_t1.has_value() && pos2_t.has_value() && pos2_t1.has_value()) {
		// 检查是否交换位置（边冲突）
		if (pos1_t->voxel == pos2_t1->voxel && pos1_t1->voxel == pos2_t->voxel) {
			voxel1 = pos1_t->voxel;
			voxel2 = pos1_t1->voxel;
			return true;
		}
	}

	return false;
}

Path CBS::planPath(int agentId, const Voxel& start, const Voxel& goal, int startTime,
				   const std::vector<Constraint>& constraints) {
	m_LastStats.totalAStarCalls++;
	return m_AStar.findPath(agentId, start, goal, startTime, constraints);
}

bool CBS::isValidSolution(const std::vector<Path>& paths) {
	auto conflicts = detectConflicts(paths);
	return conflicts.empty();
}