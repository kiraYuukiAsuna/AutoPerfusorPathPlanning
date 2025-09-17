#pragma once

#include <cmath>
#include <functional>

struct Position {
    double x, y, z;

    Position(double x = 0, double y = 0, double z = 0)
        : x(x), y(y), z(z) {}

    double distanceTo(const Position& other) const {
        return std::sqrt((x - other.x) * (x - other.x) +
                        (y - other.y) * (y - other.y) +
                        (z - other.z) * (z - other.z));
    }

    Position operator+(const Position& other) const {
        return Position(x + other.x, y + other.y, z + other.z);
    }

    Position operator-(const Position& other) const {
        return Position(x - other.x, y - other.y, z - other.z);
    }

    Position operator*(double scalar) const {
        return Position(x * scalar, y * scalar, z * scalar);
    }

    double dot(const Position& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    bool operator==(const Position& other) const {
        const double epsilon = 1e-6;
        return std::abs(x - other.x) < epsilon &&
               std::abs(y - other.y) < epsilon &&
               std::abs(z - other.z) < epsilon;
    }

    bool operator!=(const Position &other) const { return !(*this == other); }
};

struct Voxel {
	int x, y, z;

    Voxel(int x = 0, int y = 0, int z = 0) : x(x), y(y), z(z) {}

	bool operator==(const Voxel& other) const { return x == other.x && y == other.y && z == other.z; }
	
    bool operator!=(const Voxel& other) const { return !(*this == other); }
    
    // Comparison operators for std::set and sorting
    bool operator<(const Voxel& other) const {
        if (x != other.x) return x < other.x;
        if (y != other.y) return y < other.y;
        return z < other.z;
    }
    
    bool operator<=(const Voxel& other) const {
        return *this < other || *this == other;
    }
    
    bool operator>(const Voxel& other) const {
        return !(*this <= other);
    }
    
    bool operator>=(const Voxel& other) const {
        return !(*this < other);
    }
};

struct TimePoint {
    Voxel voxel;
    int time;

    TimePoint(const Voxel& v = Voxel(), int t = 0)
        : voxel(v), time(t) {}

	bool operator==(const TimePoint& other) const { return voxel == other.voxel && time == other.time; }
	
    bool operator!=(const TimePoint& other) const { return !(*this == other); }
};

namespace std {
    template <>
    struct hash<Position> {
        size_t operator()(const Position& pos) const {
            return hash<double>()(pos.x) ^ (hash<double>()(pos.y) << 1) ^ (hash<double>()(pos.z) << 2);
        }
    };

    template <>
    struct hash<Voxel> {
        size_t operator()(const Voxel& v) const {
            return hash<int>()(v.x) ^ (hash<int>()(v.y) << 1) ^ (hash<int>()(v.z) << 2);
        }
    };

    template <>
    struct hash<TimePoint> {
        size_t operator()(const TimePoint& tp) const {
            return hash<Voxel>()(tp.voxel) ^ (hash<int>()(tp.time) << 1);
        }
    };
}
