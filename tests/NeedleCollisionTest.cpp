#include <iostream>
#include <vector>
#include <optional>

#include "Core/WorldModel.hpp"
#include "Core/AgentNeedle.hpp"
#include "Core/Geometry.hpp"
#include "Planner/AStarSearch.h"

// 简易断言工具
static void require(bool cond, const char* msg) {
    if (!cond) {
        std::cerr << "[FAIL] " << msg << std::endl;
        std::exit(1);
    }
}

// 复用与CBS一致的体采样冲突检测（针-针）
static bool hasBodyConflictSampled(const WorldModel& world, const Path& p1, const Path& p2, int samplesM) {
    auto aAgent = world.getAgentNeedle(p1.agentId);
    auto bAgent = world.getAgentNeedle(p2.agentId);
    if (!aAgent || !bAgent) return false;

    int aStart = p1.getStartTime();
    int aEnd   = p1.getEndTime();
    int bStart = p2.getStartTime();
    int bEnd   = p2.getEndTime();
    int minT = std::min(aStart, bStart);
    int maxT = std::max(aEnd, bEnd);

    for (int t = minT; t < maxT; ++t) {
        bool aActiveNow   = (t >= aStart && t <= aEnd);
        bool aActiveNext  = (t + 1 >= aStart && t + 1 <= aEnd);
        bool bActiveNow   = (t >= bStart && t <= bEnd);
        bool bActiveNext  = (t + 1 >= bStart && t + 1 <= bEnd);
        if (!(aActiveNow && aActiveNext && bActiveNow && bActiveNext)) continue; // 只有双方都在[t,t+1]有效时才检测

        auto a_t = p1.getPositionAtTime(t);
        auto a_t1 = p1.getPositionAtTime(t + 1);
        auto b_t = p2.getPositionAtTime(t);
        auto b_t1 = p2.getPositionAtTime(t + 1);
        if (!a_t.has_value() || !a_t1.has_value() || !b_t.has_value() || !b_t1.has_value()) continue;

        Position a_from_w = world.voxelToWorld(a_t->voxel);
        Position a_to_w   = world.voxelToWorld(a_t1->voxel);
        Position b_from_w = world.voxelToWorld(b_t->voxel);
        Position b_to_w   = world.voxelToWorld(b_t1->voxel);

        Position a_dir = (a_t1->voxel != a_t->voxel)
                             ? geom::normalize(a_to_w - a_from_w)
                             : geom::normalize(geom::directionFromAngles(aAgent->angleHorizontal, aAgent->angleVertical));
        if (a_dir == Position(0, 0, 0)) a_dir = Position(1, 0, 0);
        Position b_dir = (b_t1->voxel != b_t->voxel)
                             ? geom::normalize(b_to_w - b_from_w)
                             : geom::normalize(geom::directionFromAngles(bAgent->angleHorizontal, bAgent->angleVertical));
        if (b_dir == Position(0, 0, 0)) b_dir = Position(1, 0, 0);

        for (int i = 0; i <= samplesM; ++i) {
            double tau = double(i) / double(samplesM);
            Position a_tip = geom::lerp(a_from_w, a_to_w, tau);
            Position b_tip = geom::lerp(b_from_w, b_to_w, tau);
            geom::Capsule ca{a_tip - a_dir * aAgent->bodyLength, a_tip, aAgent->bodyRadius};
            geom::Capsule cb{b_tip - b_dir * bAgent->bodyLength, b_tip, bAgent->bodyRadius};
            if (geom::capsuleIntersectsCapsule(ca, cb)) {
                return true;
            }
        }
    }
    return false;
}

// 顶点与边的基础冲突检查（与CBS逻辑一致）
static bool hasVertexOrEdgeConflict(const Path& p1, const Path& p2) {
    int aStart = p1.getStartTime();
    int aEnd   = p1.getEndTime();
    int bStart = p2.getStartTime();
    int bEnd   = p2.getEndTime();
    int minT = std::min(aStart, bStart);
    int maxT = std::max(aEnd, bEnd);

    for (int t = minT; t <= maxT; ++t) {
        bool aActive = (t >= aStart && t <= aEnd);
        bool bActive = (t >= bStart && t <= bEnd);
        if (aActive && bActive) {
            auto a_t = p1.getPositionAtTime(t);
            auto b_t = p2.getPositionAtTime(t);
            if (a_t.has_value() && b_t.has_value() && a_t->voxel == b_t->voxel) return true; // 顶点冲突
        }
        if (t < maxT) {
            bool aActiveNext = (t + 1 >= aStart && t + 1 <= aEnd);
            bool bActiveNext = (t + 1 >= bStart && t + 1 <= bEnd);
            if (aActive && aActiveNext && bActive && bActiveNext) {
                auto a_t = p1.getPositionAtTime(t);
                auto a_t1 = p1.getPositionAtTime(t + 1);
                auto b_t = p2.getPositionAtTime(t);
                auto b_t1 = p2.getPositionAtTime(t + 1);
                if (a_t.has_value() && a_t1.has_value() && b_t.has_value() && b_t1.has_value()) {
                    if (a_t->voxel == b_t1->voxel && a_t1->voxel == b_t->voxel) return true; // 边交换
                }
            }
        }
    }
    return false;
}

int main() {
    // 1) 构建一个 2D 平面世界（Z=1），体素尺寸=1
    WorldModel world(10, 10, 1, 1.0);

    // 2) 添加两支针，长度与半径设置为可产生碰撞
    auto agent1 = std::make_shared<AgentNeedle>(
        1, /*angleH*/ 0.0, /*angleV*/ 0.0, Voxel(2,5,0), /*radius*/ 0.4, /*length*/ 2.0,
        DyeColor::VColor1, NeedleState::Idle, -1);
    auto agent2 = std::make_shared<AgentNeedle>(
        2, /*angleH*/ 3.141592653589793, /*angleV*/ 0.0, Voxel(6,5,0), /*radius*/ 0.4, /*length*/ 2.0,
        DyeColor::VColor2, NeedleState::Idle, -1);
    world.addAgentNeedle(agent1);
    world.addAgentNeedle(agent2);

    // 3) 先用 A* 单独规划（不考虑对方）以确认会发生针-针碰撞
    AStarSearch astar(world);
    Path p1_naive = astar.findPath(1, Voxel(2,5,0), Voxel(7,5,0), 0, {});
    Path p2_naive = astar.findPath(2, Voxel(6,5,0), Voxel(1,5,0), 0, {});

    require(!p1_naive.empty() && !p2_naive.empty(), "Naive A* paths should exist");
    bool naiveBodyConflict = hasBodyConflictSampled(world, p1_naive, p2_naive, /*samplesM*/ 5);
    require(naiveBodyConflict, "Naive independent plans should have needle-body collision");

    // 4) 让第二支针延迟出发，验证碰撞被消除（模拟CBS插入等待的效果）
    int safeDelay = p1_naive.getDuration() + 1; // 确保时间窗不重叠，避免顶点/边冲突
    Path p2_delayed = astar.findPath(2, Voxel(6,5,0), Voxel(1,5,0), /*startTime*/ safeDelay, {});
    require(!p2_delayed.empty(), "Delayed A* path for agent2 should exist");

    // 5) 验证：无顶点/边冲突，且无针-针体冲突
    require(!hasVertexOrEdgeConflict(p1_naive, p2_delayed), "Delayed plan should not have vertex/edge conflicts");
    bool finalBodyConflict = hasBodyConflictSampled(world, p1_naive, p2_delayed, /*samplesM*/ 6);
    require(!finalBodyConflict, "Delayed plan must be needle-body collision free");

    std::cout << "[PASS] Needle-needle collision detected (naive) and avoided by delaying agent2 (delay="
              << safeDelay << ")." << std::endl;
    return 0;
}
