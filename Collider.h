#ifndef PHYSICSENGINE_COLLIDER_H
#define PHYSICSENGINE_COLLIDER_H

#include <SFML/System/Vector2.hpp>
#include <memory>

enum class ColliderType {
    Circle,
    AABB
};

struct Collider {
    ColliderType type;
    
    explicit Collider(ColliderType t) : type(t) {}
    virtual ~Collider() = default;
};

struct CircleCollider : Collider {
    float radius;
    
    explicit CircleCollider(float r) 
        : Collider(ColliderType::Circle), radius(r) {}
};

struct AABBCollider : Collider {
    sf::Vector2f halfExtents;  // Half-width and half-height
    
    explicit AABBCollider(sf::Vector2f size) 
        : Collider(ColliderType::AABB), halfExtents(size * 0.5f) {}
    
    AABBCollider(float width, float height) 
        : Collider(ColliderType::AABB), halfExtents({width * 0.5f, height * 0.5f}) {}
};

// Helper to create colliders
inline std::unique_ptr<Collider> MakeCircleCollider(float radius) {
    return std::make_unique<CircleCollider>(radius);
}

inline std::unique_ptr<Collider> MakeAABBCollider(float width, float height) {
    return std::make_unique<AABBCollider>(width, height);
}

inline std::unique_ptr<Collider> MakeAABBCollider(sf::Vector2f size) {
    return std::make_unique<AABBCollider>(size);
}

#endif //PHYSICSENGINE_COLLIDER_H

