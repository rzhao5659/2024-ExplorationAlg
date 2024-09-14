#pragma once
#include <memory>

// Combine hash values. Taken from boost hash combine.
template <class T>
inline void hash_combine(std::size_t& seed, const T& v) {
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

/**
 * Represents a 2D point.
 */
struct cell2d_t {
    int x;
    int y;
    bool operator==(const cell2d_t& other) const { return x == other.x && y == other.y; }
    // Hash function for cell2d_t
    struct hash {
        std::size_t operator()(const cell2d_t& p) const {
            std::size_t hash = 0;
            hash_combine(hash, p.x);
            hash_combine(hash, p.y);
            return hash;
        };
    };
};
