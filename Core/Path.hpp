#pragma once

#include "Position.hpp"
#include <vector>
#include <optional>
#include <cmath>

struct Path {
	std::vector<TimePoint> wayPoints;
	double cost;
	int agentId;

	Path(int agentId=-1) : cost(0), agentId(agentId) { }

	void addWayPoint(const TimePoint& p) { wayPoints.push_back(p); }

	int getDuration() const { return wayPoints.back().time - wayPoints.front().time; }

	int getStartTime() const { return wayPoints.front().time; }

	int getEndTime() const { return wayPoints.back().time; }

	size_t size() const { return wayPoints.size(); }

	bool empty() const { return wayPoints.empty(); }

	void clear() { wayPoints.clear(); cost = 0; }

	std::optional<TimePoint> getPositionAtTime(int time) const {
		if (wayPoints.empty()) {
			return std::nullopt;
		}
		if (time <= wayPoints.front().time)
            return wayPoints.front();
        if (time >= wayPoints.back().time)
            return wayPoints.back();
        for (size_t i = 0; i < wayPoints.size(); i++) {
            if (wayPoints[i].time == time)
                return wayPoints[i];
        }
		return std::nullopt;
	}

	void addWaitActionsAtEnd(int numWaits) {
        if (wayPoints.empty() || numWaits <= 0) return;
        TimePoint last = wayPoints.back();
        for (int i = 1; i <= numWaits; i++) {
            wayPoints.push_back(TimePoint(last.voxel, last.time + i));
        }
    }

	// 计算路径长度（基于体素距离）
	double getTotalLength() const {
        if (wayPoints.size() < 2) return 0.0;
        double totalLength = 0.0;
        for (size_t i = 1; i < wayPoints.size(); i++) {
            // 使用曼哈顿距离或欧几里得距离计算体素间距离
            Voxel v1 = wayPoints[i-1].voxel;
            Voxel v2 = wayPoints[i].voxel;
            double dx = v2.x - v1.x;
            double dy = v2.y - v1.y;
            double dz = v2.z - v1.z;
            totalLength += std::sqrt(dx*dx + dy*dy + dz*dz);
        }
        return totalLength;
    }
};
