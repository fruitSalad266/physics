#ifndef PHYSICSENGINE_OBJECT_H
#define PHYSICSENGINE_OBJECT_H

#include <SFML/System/Vector2.hpp>
#include <memory>
#include "Collider.h"

struct Object {
    sf::Vector2f position;
    sf::Vector2f oldPosition;    // Verlet
    sf::Vector2f acceleration;   // acceleration for this frame
    //vertlet: determine v directly, without needing to store

    float mass = 1.0f;
    float bounciness = 0.7f;
    bool isStatic = false;
    
    std::unique_ptr<Collider> collider;
    
    // Initialize oldPosition to match position (object starts at rest)
    void InitVerlet() {
        oldPosition = position;
        acceleration = {0.f, 0.f};
    }
    
    // Get implicit velocity from Verlet state
    sf::Vector2f GetVelocity(float dt) const {
        if (dt <= 0.f) return {0.f, 0.f};
        return (position - oldPosition) / dt;
    }
    
    // Set velocity by adjusting oldPosition
    void SetVelocity(sf::Vector2f vel, float dt) {
        oldPosition = position - vel * dt;
    }
    
    // Convenience methods
    void SetCircleCollider(float radius) {
        collider = MakeCircleCollider(radius);
    }
    
    void SetAABBCollider(float width, float height) {
        collider = MakeAABBCollider(width, height);
    }
    
    void SetAABBCollider(sf::Vector2f size) {
        collider = MakeAABBCollider(size);
    }
    
    // Helper to get collider as specific type (returns nullptr if wrong type)
    CircleCollider* GetCircleCollider() const {
        if (collider && collider->type == ColliderType::Circle) {
            return static_cast<CircleCollider*>(collider.get());
        }
        return nullptr;
    }
    
    AABBCollider* GetAABBCollider() const {
        if (collider && collider->type == ColliderType::AABB) {
            return static_cast<AABBCollider*>(collider.get());
        }
        return nullptr;
    }
};

#endif //PHYSICSENGINE_OBJECT_H
