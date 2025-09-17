#pragma once

#include "Position.hpp"
#include <string>
#include <vector>

enum class NeedleState { Idle, Working, Injecting, Blocking, Error };

enum class DyeColor {
	VColor1,
	VColor2,
	VColor3,
	VColor4,
	VColor5,
	VColor6,
};

struct AgentNeedle {
	int agentId;
	
	double angleHorizontal; // in radians
	double angleVertical;	// in radians
	
	Voxel tipPosition;

	double bodyRadius;
	double bodyLength;

	DyeColor dyeColor;
	NeedleState state;

	int targetTaskId;

    AgentNeedle(int agentId, double angleHorizontal, double angleVertical,
        Voxel tipPosition, double bodyRadius, double bodyLength,
        DyeColor dyeColor, NeedleState state, int targetTaskId)
        : agentId(agentId), angleHorizontal(angleHorizontal), angleVertical(angleVertical),
          tipPosition(tipPosition), bodyRadius(bodyRadius), bodyLength(bodyLength),
          dyeColor(dyeColor), state(state), targetTaskId(targetTaskId) {}

	std::vector<float> getColorRGB() const {
        switch (dyeColor) {
            case DyeColor::VColor1: return {1.0f, 0.0f, 0.0f}; // Red
            case DyeColor::VColor2: return {0.0f, 1.0f, 0.0f}; // Green
            case DyeColor::VColor3: return {0.0f, 0.0f, 1.0f}; // Blue
            case DyeColor::VColor4: return {1.0f, 1.0f, 0.0f}; // Yellow
            case DyeColor::VColor5: return {1.0f, 0.65f, 0.0f}; // Orange
            case DyeColor::VColor6: return {0.5f, 0.0f, 0.5f}; // Purple
            default: return {1.0f, 1.0f, 1.0f}; // White as default
        }
	}

	std::string getColorString() const {
        switch (dyeColor) {
            case DyeColor::VColor1: return "Red";
            case DyeColor::VColor2: return "Green";
            case DyeColor::VColor3: return "Blue";
            case DyeColor::VColor4: return "Yellow";
            case DyeColor::VColor5: return "Orange";
            case DyeColor::VColor6: return "Purple";
            default: return "Unknown";
        }
	}

	std::string getStateString() const {
        switch (state) {
            case NeedleState::Idle: return "Idle";
            case NeedleState::Working: return "Working";
            case NeedleState::Injecting: return "Injecting";
            case NeedleState::Blocking: return "Blocking";
            case NeedleState::Error: return "Error";
            default: return "Unknown";
        }
	}
	
};
