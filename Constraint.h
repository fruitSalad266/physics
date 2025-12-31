#ifndef PHYSICSENGINE_CONSTRAINT_H
#define PHYSICSENGINE_CONSTRAINT_H

#include <SFML/System/Vector2.hpp>
#include "Object.h"
#include <cmath>

/**
 * 
 * Adj positions instead of applying forces
 * aka "Position-Based Dynamics" 
 * 
 * Correct positions after physics to maintain relationships
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
    
    // Position adjuster
    virtual void Solve() = 0;
};

/**
 * Rigid link
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
    float restLength;      //dist to maintain
    float stiffness;       // 1.0 = fully rigid, <1.0 = slightly elastic
    
    // Explicit length constructor
    DistanceConstraint(Object* a, Object* b, float length, float stiff)
        : Constraint(ConstraintType::Distance), 
          objA(a), objB(b), restLength(length), stiffness(stiff) {}
    
    //Calc rest length from current positions
    DistanceConstraint(Object* a, Object* b)
        : Constraint(ConstraintType::Distance), objA(a), objB(b), stiffness(1.0f) {
        sf::Vector2f diff = objB->position - objA->position;
        restLength = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    
    void Solve() override {
        sf::Vector2f diff = objB->position - objA->position;
        float currentLength = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        
        if (currentLength < 0.0001f) return;  //No div by zero
        
        //Correct by this
        float error = currentLength - restLength;
        sf::Vector2f direction = diff / currentLength;
        sf::Vector2f correction = direction * (error * stiffness);
        
        // Distribute correction based on which objects can move
        // If one is static, the other takes full correction
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
 * SPRING CONSTRAINT (Soft Link)
 * 
 * hooke's Law: F = -k * x (force proportional to displacement)
 * Oscillation allowed
 */
struct SpringConstraint : Constraint {
    Object* objA;
    Object* objB;
    float restLength;      // Natural length of spring
    float stiffness;       // Spring constant 
    float damping;         // Energy loss 
    
    SpringConstraint(Object* a, Object* b, float length, float stiff = 0.5f, float damp = 0.1f)
        : Constraint(ConstraintType::Spring),
          objA(a), objB(b), restLength(length), stiffness(stiff), damping(damp) {}
    
    SpringConstraint(Object* a, Object* b, float stiff, float damp)
        : Constraint(ConstraintType::Spring), objA(a), objB(b), stiffness(stiff), damping(damp) {
        sf::Vector2f diff = objB->position - objA->position;
        restLength = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    }
    
    void Solve() override {
        sf::Vector2f diff = objB->position - objA->position;
        float currentLength = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        
        if (currentLength < 0.0001f) return;
        
        //(Hooke's Law)
        float displacement = currentLength - restLength;
        sf::Vector2f direction = diff / currentLength;
        
        // velocities 
        sf::Vector2f velA = objA->position - objA->oldPosition;
        sf::Vector2f velB = objB->position - objB->oldPosition;
        sf::Vector2f relativeVel = velB - velA;
        
        // Damping force 
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
 * Rotation allowed - pivot points.
 * Position is equal to anchor for each solve.
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
        //Force pos to error
        obj->position = anchor;
    }
    
    //Move anchor if dragging/other
    void SetAnchor(sf::Vector2f newPos) {
        anchor = newPos;
    }
};

#endif //PHYSICSENGINE_CONSTRAINT_H

