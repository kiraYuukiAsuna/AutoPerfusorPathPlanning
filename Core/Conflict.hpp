#pragma once

#include "Position.hpp"
#include "Constraint.hpp"
#include <vector>

enum class ConflictType {
    Vertex,      // Two agents at same voxel at same time
    Edge         // Two agents swapping voxels
};

struct Conflict {
    int agent1;
    int agent2;  // -1 if conflict with environment/camera
    Voxel voxel1;
    Voxel voxel2; // Only used for Edge conflicts
    int time;
    ConflictType type;

    Conflict(int agent1, int agent2, const Voxel& voxel, int time, ConflictType type = ConflictType::Vertex)
        : agent1(agent1), agent2(agent2), voxel1(voxel), voxel2(-1, -1, -1), time(time), type(type) {}
	
    Conflict(int agent1, int agent2, const Voxel& v1, const Voxel& v2, int time, ConflictType type = ConflictType::Edge)
        : agent1(agent1), agent2(agent2), voxel1(v1), voxel2(v2), time(time), type(type) {}


	bool operator==(const Conflict& other) const {
        return agent1 == other.agent1 && agent2 == other.agent2 && voxel1 == other.voxel1 &&
               voxel2 == other.voxel2 && time == other.time && type == other.type;
    }

	bool operator!=(const Conflict& other) const { return !(*this == other); }

	std::vector<Constraint> toConstraints() const {
        std::vector<Constraint> constraints;
        if (type == ConflictType::Vertex) {
            constraints.emplace_back(agent1, voxel1, time, ConstraintType::Vertex);
            if (agent2 != -1) {
                constraints.emplace_back(agent2, voxel1, time, ConstraintType::Vertex);
            }
        } else if (type == ConflictType::Edge) {
            constraints.emplace_back(agent1, voxel1, voxel2, time, -1, ConstraintType::Edge);
            if (agent2 != -1) {
                constraints.emplace_back(agent2, voxel2, voxel1, time, -1, ConstraintType::Edge);
            }
        }
        return constraints;
    }
	
};
