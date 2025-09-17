#pragma once

#include "Core/Path.hpp"
#include "Core/Constraint.hpp"
#include "Core/WorldModel.hpp"

struct CBSNode {
	std::vector<Path> solution;
	std::vector<Constraint> constraints;
	double cost;
	int nodeId;

	CBSNode() : cost(0), nodeId(0) {}

	bool operator>(const CBSNode& other) const {
        return cost > other.cost;
    }
};

class CBS {
public:
	
private:
	
};

