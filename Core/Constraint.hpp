#pragma once

#include "Position.hpp"

enum class ConstraintType {
    Vertex, // Agent cannot be at voxel at time
    Edge    // Agent cannot move from voxel1 to voxel2 at time
};


struct Constraint {
	int agentId;
	Voxel voxel1;
	Voxel voxel2;	 // only for edge constraints
	int time;
	int endTime; // for interval constraints
	ConstraintType type;
	
	Constraint(int agentId, Voxel voxel, int time, ConstraintType type = ConstraintType::Vertex)
        : agentId(agentId), voxel1(voxel), time(time), type(type), endTime(-1), voxel2(-1, -1, -1) {}

    Constraint(int agentId, Voxel v1, Voxel v2, int time, int endTime = -1, ConstraintType type = ConstraintType::Edge)
        : agentId(agentId), voxel1(v1), voxel2(v2), time(time), endTime(endTime), type(type) {}

    bool operator==(const Constraint& other) const {
        return agentId == other.agentId && voxel1 == other.voxel1 && voxel2 == other.voxel2
               && time == other.time && endTime == other.endTime && type == other.type;
	}

	bool operator!=(const Constraint& other) const { return !(*this == other); }

    bool violates(int agentId, Voxel voxel, int time) const {
        if (this->agentId != agentId) return false;
        if (type == ConstraintType::Vertex) {
            return voxel1 == voxel && time == this->time;
        }else if (type == ConstraintType::Edge) {
            return (time == this->time || time == endTime) && (voxel1 == voxel || voxel2 == voxel);
        }
        return false;
    }
	
	bool violates(int agentId, Voxel from, Voxel to, int time, int endTime) const {
        if (this->agentId != agentId) return false;
        if (type == ConstraintType::Vertex) {
            return voxel1 == to && time <= endTime && this->endTime >= time;
        }else if (type == ConstraintType::Edge) {
            bool voxelMatch = (voxel1 == from && voxel2 == to);
            bool timeOverlap = false;
            
            if (this->endTime >= 0) {
                timeOverlap = !(endTime < this->time || time > this->endTime);
            } else {
                timeOverlap = (this->time >= time && this->time <= endTime);
            }
            
            return voxelMatch && timeOverlap;
        }
        return false;
    }
};
