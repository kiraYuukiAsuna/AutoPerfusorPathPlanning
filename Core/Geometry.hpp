#pragma once

#include <cmath>
#include <algorithm>
#include "Position.hpp"

// 基本向量/点工具
namespace geom {

inline Position normalize(const Position& v) {
    double n = std::sqrt(v.x*v.x + v.y*v.y + v.z*v.z);
    if (n < 1e-9) return Position(0,0,0);
    return Position(v.x/n, v.y/n, v.z/n);
}

inline double clamp(double v, double lo, double hi) { return std::max(lo, std::min(v, hi)); }

inline double dot(const Position& a, const Position& b) { return a.x*b.x + a.y*b.y + a.z*b.z; }

inline Position lerp(const Position& a, const Position& b, double t) {
    return Position(a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t, a.z + (b.z-a.z)*t);
}

// 由水平角/垂直角转换为单位方向向量（右手系，水平角绕Z，垂直角仰角）
inline Position directionFromAngles(double angleHorizontal, double angleVertical) {
    double ch = std::cos(angleHorizontal), sh = std::sin(angleHorizontal);
    double cv = std::cos(angleVertical), sv = std::sin(angleVertical);
    // 水平面方向 (x,y) 乘以 cos(垂直)，z 为 sin(垂直)
    return Position(cv*ch, cv*sh, sv);
}

// 线段最近距离：段 ab 与段 cd 的最短距离平方
inline double segmentSegmentDist2(const Position& a, const Position& b,
                                  const Position& c, const Position& d) {
    Position u = b - a;
    Position v = d - c;
    Position w = a - c;
    double aU = dot(u,u);         // |u|^2
    double bUV = dot(u,v);
    double cV = dot(v,v);         // |v|^2
    double dUw = dot(u,w);
    double eVw = dot(v,w);
    double D = aU*cV - bUV*bUV;
    double sc, sN, sD = D;  // sc = sN / sD
    double tc, tN, tD = D;  // tc = tN / tD

    const double EPS = 1e-9;
    if (D < EPS) { // 平行或接近平行
        sN = 0.0;   sD = 1.0;
        tN = eVw;   tD = cV;
    } else {
        sN = (bUV*eVw - cV*dUw);
        tN = (aU*eVw - bUV*dUw);
        if (sN < 0) { sN = 0; tN = eVw; tD = cV; }
        else if (sN > sD) { sN = sD; tN = eVw + bUV; tD = cV; }
    }
    if (tN < 0) {
        tN = 0;
        if (-dUw < 0) sN = 0; else if (-dUw > aU) sN = sD; else { sN = -dUw; sD = aU; }
    } else if (tN > tD) {
        tN = tD;
        double tmp = -dUw + bUV;
        if (tmp < 0) sN = 0; else if (tmp > aU) sN = sD; else { sN = tmp; sD = aU; }
    }

    sc = (std::abs(sN) < EPS ? 0.0 : sN / sD);
    tc = (std::abs(tN) < EPS ? 0.0 : tN / tD);

    Position dP = w + u*sc - v*tc; // 最近点之间的向量
    return dot(dP, dP);
}

// 点到 AABB 的最近距离平方
inline double pointAABBDist2(const Position& p, const Position& bmin, const Position& bmax) {
    double dx = (p.x < bmin.x) ? bmin.x - p.x : (p.x > bmax.x ? p.x - bmax.x : 0.0);
    double dy = (p.y < bmin.y) ? bmin.y - p.y : (p.y > bmax.y ? p.y - bmax.y : 0.0);
    double dz = (p.z < bmin.z) ? bmin.z - p.z : (p.z > bmax.z ? p.z - bmax.z : 0.0);
    return dx*dx + dy*dy + dz*dz;
}

// 线段到 AABB 最近距离平方（基于离散近似：投影+裁剪）
inline double segmentAABBDist2(const Position& a, const Position& b,
                               const Position& bmin, const Position& bmax) {
    // 粗略而快：对若干采样点估计下界，足够用于预筛；需要更精确可加入线段-盒精确距离。
    const int S = 5;
    double best = 1e100;
    for (int i=0; i<=S; ++i) {
        double t = double(i)/S;
        Position p = lerp(a,b,t);
        best = std::min(best, pointAABBDist2(p, bmin, bmax));
        if (best < 1e-12) return 0.0;
    }
    return best;
}

// 胶囊体（线段 + 半径）
struct Capsule {
    Position a; // 线段端点1（针尾）
    Position b; // 线段端点2（针尖）
    double r;   // 半径
};

inline bool capsuleIntersectsCapsule(const Capsule& c1, const Capsule& c2) {
    double d2 = segmentSegmentDist2(c1.a, c1.b, c2.a, c2.b);
    double rr = c1.r + c2.r;
    return d2 <= rr*rr;
}

inline bool capsuleIntersectsAABB(const Capsule& c, const Position& bmin, const Position& bmax) {
    double d2 = segmentAABBDist2(c.a, c.b, bmin, bmax);
    return d2 <= c.r*c.r;
}

} // namespace geom
