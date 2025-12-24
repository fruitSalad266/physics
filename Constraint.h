#ifndef PHYSICSENGINE_CONSTRAINT_H
#define PHYSICSENGINE_CONSTRAINT_H

#include <SFML/System/Vector2.hpp>
#include "Object.h"
#include <cmath>

/**
 * CONSTRAINT PHYSICS CONCEPTS:
 * 
 * Constraints are rules that objects must obey. Instead of applying forces,
 * we directly adjust positions to satisfy the constraint. This is called
 * "Position-Based Dynamics" and works perfectly with Verlet integration.
 * 
 * The key idea: after physics moves objects, constraints "correct" positions
 * to maintain relationships (like fixed distances between points).
 */

enum class ConstraintType {
    Distance,
    Spring,
    Pin
};

// Base constraint interface
struct Constraint {
    ConstraintType type;
    
    explicit Constraint(ConstraintType t) : type(t) {}
    virtual ~Constraint() = default;
    
    // Solve the constraint by adjusting positions
    virtual void Solve() = 0;
};

/**
 * DISTANCE CONSTRAINT (Rigid Link)
 * 
 * Maintains a fixed distance between two objects - like a rigid rod.
 * Used for: pendulum arms, rigid body connections, chains
 * 
 * Algorithm:
 * 1. Calculate current distance between objects
 * 2. Find the difference from rest length
 * 3. Move each object along the connecting line to fix the error
 *    (static objects don't move, so dynamic ones move more)
 */
struct DistanceConstraint : Constraint {
    Object* objA;
    Object* objB;
    float restLength;      // The distance we want to maintain
    float stiffness;       // 1.0 = fully rigid, <1.0 = slightly elastic
    
    // Explicit length constructor
    DistanceConstraint(Object* a, Object* b, float length, float stiff)
        : Constraint(ConstraintType::Distance), 
          objA(a), objB(b), restLength(length), stiffness(stiff) {}
    
    // Auto-calculate rest length from current positions
    DistanceConstraint(Object* a, Object* b)
        : Constraint(ConstraintType::Distance), objA(a), objB(b), stiffness(1.0f) {
        sf::Vector2f diff = objB->position - objA->position;
        restLength = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    
    void Solve() override {
        sf::Vector2f diff = objB->position - objA->position;
        float currentLength = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        
        if (currentLength < 0.0001f) return;  // Avoid division by zero
        
        // How much we need to correct
        float error = currentLength - restLength;
        
        // Direction to push/pull (normalized)
        sf::Vector2f direction = diff / currentLength;
        
        // Correction amount (scaled by stiffness)
        sf::Vector2f correction = direction * (error * stiffness);
        
        // Distribute correction based on which objects can move
        // If one is static, the other takes full correction
        if (objA->isStatic) {
            objB->position -= correction;
        } else if (objB->isStatic) {
            objA->position += correction;
        } else {
            // Both dynamic: split 50/50
            objA->position += correction * 0.5f;
            objB->position -= correction * 0.5f;
        }
    }
};

/**
 * SPRING CONSTRAINT (Soft Link)
 * 
 * Like distance constraint but with elasticity and damping.
 * Uses Hooke's Law: F = -k * x (force proportional to displacement)
 * 
 * Unlike rigid distance constraint, springs allow oscillation.
 * Damping reduces oscillation over time (prevents infinite bouncing).
 * 
 * Used for: soft bodies, cloth simulation, bouncy connections
 */
struct SpringConstraint : Constraint {
    Object* objA;
    Object* objB;
    float restLength;      // Natural length of spring
    float stiffness;       // Spring constant (higher = stiffer)
    float damping;         // Energy loss (higher = less bouncy)
    
    SpringConstraint(Object* a, Object* b, float length, float stiff = 0.5f, float damp = 0.1f)
        : Constraint(ConstraintType::Spring),
          objA(a), objB(b), restLength(length), stiffness(stiff), damping(damp) {}
    
    // Auto-calculate rest length
    SpringConstraint(Object* a, Object* b, float stiff, float damp)
        : Constraint(ConstraintType::Spring), objA(a), objB(b), stiffness(stiff), damping(damp) {
        sf::Vector2f diff = objB->position - objA->position;
        restLength = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    
    void Solve() override {
        sf::Vector2f diff = objB->position - objA->position;
        float currentLength = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        
        if (currentLength < 0.0001f) return;
        
        // Spring force (Hooke's Law)
        float displacement = currentLength - restLength;
        sf::Vector2f direction = diff / currentLength;
        
        // Get velocities (for damping)
        sf::Vector2f velA = objA->position - objA->oldPosition;
        sf::Vector2f velB = objB->position - objB->oldPosition;
        sf::Vector2f relativeVel = velB - velA;
        
        // Damping force (opposes relative motion along spring)
        float dampingForce = (relativeVel.x * direction.x + relativeVel.y * direction.y) * damping;
        
        // Total correction = spring + damping
        sf::Vector2f correction = direction * (displacement * stiffness + dampingForce);
        
        if (objA->isStatic) {
            objB->position -= correction;
        } else if (objB->isStatic) {
            objA->position += correction;
        } else {
            objA->position += correction * 0.5f;
            objB->position -= correction * 0.5f;
        }
    }
};

/**
 * PIN CONSTRAINT (Anchor)
 * 
 * Locks an object to a fixed point in space.
 * The object can still rotate around this point (if connected to others).
 * 
 * Used for: pendulum pivot points, fixed attachment points
 * 
 * This is the simplest constraint: just set position = anchor each solve.
 */
struct PinConstraint : Constraint {
    Object* obj;
    sf::Vector2f anchor;   // Fixed point in world space
    
    PinConstraint(Object* o, sf::Vector2f point)
        : Constraint(ConstraintType::Pin), obj(o), anchor(point) {}
    
    // Pin to current position
    explicit PinConstraint(Object* o)
        : Constraint(ConstraintType::Pin), obj(o), anchor(o->position) {}
    
    void Solve() override {
        // Simply force the object to the anchor point
        obj->position = anchor;
    }
    
    // Allow moving the anchor (for interactive dragging)
    void SetAnchor(sf::Vector2f newPos) {
        anchor = newPos;
    }
};

#endif //PHYSICSENGINE_CONSTRAINT_H

