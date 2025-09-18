#include "AStarSearch.h"

#include <algorithm>
#include <cmath>


AStarSearch::AStarSearch(const WorldModel& worldModel)
	: m_WorldModel(worldModel),
	  m_MaxSearchTime(100000),
	  m_MaxSearchNodes(5000000),
	  m_HeuristicType(HeuristicType::Euclidean),
	  m_NeighborType(NeighborType::TwentySix) {}

Path AStarSearch::findPath(int agentId, const Voxel& start, const Voxel& goal, int startTime,
						   const std::vector<Constraint>& constraints) {
	// 重置统计信息
	m_LastStats = SearchStats{};

	// 检查起点和终点是否有效
	if (!m_WorldModel.isVoxelValid(start) || !m_WorldModel.isVoxelValid(goal)) {
		return Path(agentId);  // 返回空路径
	}

	if (isVoxelObstructed(start) || isVoxelObstructed(goal)) {
		return Path(agentId);  // 返回空路径
	}

	// 开放列表和关闭列表
	std::priority_queue<std::shared_ptr<AStarNode>, std::vector<std::shared_ptr<AStarNode>>,
						std::function<bool(const std::shared_ptr<AStarNode>&, const std::shared_ptr<AStarNode>&)>>
		openList([](const auto& a, const auto& b) { return *a > *b; });

	std::unordered_set<TimePoint> closedList;
	std::unordered_map<TimePoint, std::shared_ptr<AStarNode>> allNodes;

	// 创建起始节点
	TimePoint startTimePoint(start, startTime);
	double hCost = calculateHeuristic(start, goal);
	auto startNode = std::make_shared<AStarNode>(startTimePoint, 0, hCost);

	openList.push(startNode);
	allNodes[startTimePoint] = startNode;

	int searchedNodes = 0;

	while (!openList.empty() && searchedNodes < m_MaxSearchNodes) {
		auto current = openList.top();
		openList.pop();
		searchedNodes++;

		// 检查是否到达目标
		if (current->timePoint.voxel == goal) {
			Path result = reconstructPath(current, agentId);
			m_LastStats.pathFound = true;
			m_LastStats.nodesExplored = searchedNodes;
			m_LastStats.pathLength = static_cast<int>(result.size());
			m_LastStats.pathCost = result.cost;
			return result;
		}

		// 如果超过最大搜索时间，停止搜索
		if (current->timePoint.time > startTime + m_MaxSearchTime) {
			continue;
		}

		closedList.insert(current->timePoint);

		// 探索邻居节点
		for (const auto& neighborTimePoint : getNeighbors(current->timePoint)) {
			if (closedList.count(neighborTimePoint)) continue;

			if (!isValidMove(agentId, current->timePoint, neighborTimePoint, constraints)) {
				continue;
			}

			double moveCost = getMoveCost(current->timePoint.voxel, neighborTimePoint.voxel);
			double tentativeGCost = current->gCost + moveCost;

			auto it = allNodes.find(neighborTimePoint);
			if (it == allNodes.end()) {
				// 新节点
				double hCost = calculateHeuristic(neighborTimePoint.voxel, goal);
				auto neighborNode = std::make_shared<AStarNode>(neighborTimePoint, tentativeGCost, hCost, current);

				openList.push(neighborNode);
				allNodes[neighborTimePoint] = neighborNode;
			} else if (tentativeGCost < it->second->gCost) {
				// 找到更好的路径
				it->second->gCost = tentativeGCost;
				it->second->fCost = tentativeGCost + it->second->hCost;
				it->second->parent = current;
				openList.push(it->second);
			}
		}
	}

	// 未找到路径，更新统计信息
	m_LastStats.nodesExplored = searchedNodes;
	m_LastStats.pathFound = false;
	return Path(agentId);
}

double AStarSearch::calculateHeuristic(const Voxel& from, const Voxel& to) const {
	double dx = std::abs(to.x - from.x);
	double dy = std::abs(to.y - from.y);
	double dz = std::abs(to.z - from.z);

	switch (m_HeuristicType) {
		case HeuristicType::Manhattan:
			return dx + dy + dz;

		case HeuristicType::Euclidean:
			return std::sqrt(dx * dx + dy * dy + dz * dz);

		case HeuristicType::Chebyshev:
			return std::max({dx, dy, dz});

		default:
			return std::sqrt(dx * dx + dy * dy + dz * dz);
	}
}

std::vector<TimePoint> AStarSearch::getNeighbors(const TimePoint& current) const {
	std::vector<TimePoint> neighbors;
	std::vector<std::tuple<int, int, int>> directions;

	if (m_NeighborType == NeighborType::Six) {
		// 6-连通：上下左右前后
		directions = {
			{1, 0, 0}, {-1, 0, 0},	// x方向
			{0, 1, 0}, {0, -1, 0},	// y方向
			{0, 0, 1}, {0, 0, -1}	// z方向
		};
	} else if (m_NeighborType == NeighborType::TwentySix) {
		// 26-连通：包括对角线移动
		for (int dx = -1; dx <= 1; dx++) {
			for (int dy = -1; dy <= 1; dy++) {
				for (int dz = -1; dz <= 1; dz++) {
					if (dx == 0 && dy == 0 && dz == 0) continue;  // 跳过当前位置
					directions.emplace_back(dx, dy, dz);
				}
			}
		}
	} else if (m_NeighborType == NeighborType::Custom) {
		directions = m_CustomOffsets;
	}

	// 添加移动邻居
	for (const auto& [dx, dy, dz] : directions) {
		Voxel newVoxel(current.voxel.x + dx, current.voxel.y + dy, current.voxel.z + dz);
		if (m_WorldModel.isVoxelValid(newVoxel) && !isVoxelObstructed(newVoxel)) {
			neighbors.emplace_back(newVoxel, current.time + 1);
		}
	}

	// 添加等待动作（在当前位置停留）
	neighbors.emplace_back(current.voxel, current.time + 1);

	return neighbors;
}

bool AStarSearch::isValidMove(int agentId, const TimePoint& from, const TimePoint& to,
							  const std::vector<Constraint>& constraints) const {
	// 检查目标体素是否被阻挡
	if (isVoxelObstructed(to.voxel)) {
		return false;
	}

	// 检查是否违反约束
	if (violatesConstraints(agentId, from, to, constraints)) {
		return false;
	}

	return true;
}

bool AStarSearch::isVoxelObstructed(const Voxel& voxel) const { return m_WorldModel.hasStaticObstacle(voxel); }

Path AStarSearch::reconstructPath(std::shared_ptr<AStarNode> goalNode, int agentId) const {
	Path path(agentId);
	std::vector<TimePoint> waypoints;

	// 从目标节点向后追溯到起始节点
	auto current = goalNode;
	while (current != nullptr) {
		waypoints.push_back(current->timePoint);
		current = current->parent;
	}

	// 反转路径（从起点到终点）
	std::reverse(waypoints.begin(), waypoints.end());

	// 添加路径点并计算代价
	double totalCost = 0;
	for (size_t i = 0; i < waypoints.size(); i++) {
		path.addWayPoint(waypoints[i]);

		if (i > 0) {
			totalCost += getMoveCost(waypoints[i - 1].voxel, waypoints[i].voxel);
		}
	}

	path.cost = totalCost;
	return path;
}

bool AStarSearch::violatesConstraints(int agentId, const TimePoint& from, const TimePoint& to,
									  const std::vector<Constraint>& constraints) const {
	for (const auto& constraint : constraints) {
		if (constraint.agentId != agentId) continue;

		if (constraint.type == ConstraintType::Vertex) {
			// 顶点约束：不能在特定时间占用特定体素
			if (constraint.violates(agentId, to.voxel, to.time)) {
				return true;
			}
		} else if (constraint.type == ConstraintType::Edge) {
			// 边约束：不能在特定时间从一个体素移动到另一个体素
			if (constraint.violates(agentId, from.voxel, to.voxel, from.time, to.time)) {
				return true;
			}
		}
	}
	return false;
}

double AStarSearch::getMoveCost(const Voxel& from, const Voxel& to) const {
	if (from == to) {
		return 1.0;	 // 等待动作的代价
	}

	double dx = std::abs(to.x - from.x);
	double dy = std::abs(to.y - from.y);
	double dz = std::abs(to.z - from.z);

	// 使用欧几里得距离作为移动代价
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}