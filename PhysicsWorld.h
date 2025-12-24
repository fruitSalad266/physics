#ifndef PHYSICSENGINE_PHYSICSWORLD_H
#define PHYSICSENGINE_PHYSICSWORLD_H

#include <vector>
#include <memory>
#include <SFML/System/Vector2.hpp>
#include "Object.h"
#include "Constraint.h"

class PhysicsWorld {
private:
    std::vector<Object*> m_objects;
    std::vector<std::unique_ptr<Constraint>> m_constraints;
    sf::Vector2f m_gravity;
    int m_constraintIterations = 4;  // More iterations = more stable

    // Collision resolution methods
    void ResolveCollision(Object* objA, Object* objB);
    void ResolveCircleCircle(Object* objA, Object* objB);
    void ResolveCircleAABB(Object* circle, Object* box);
    void ResolveAABBAABB(Object* objA, Object* objB);
    
    // Constraint solving
    void SolveConstraints();

public:
    PhysicsWorld();

    void AddObject(Object* object);
    void RemoveObject(Object* object);
    
    // Constraint management - returns raw pointer for reference, world owns the constraint
    Constraint* AddConstraint(std::unique_ptr<Constraint> constraint);
    void RemoveConstraint(Constraint* constraint);
    
    // Convenience methods to create common constraints
    DistanceConstraint* AddDistanceConstraint(Object* a, Object* b, float length = -1.f);
    SpringConstraint* AddSpringConstraint(Object* a, Object* b, float stiffness = 0.5f, float damping = 0.1f);
    PinConstraint* AddPinConstraint(Object* obj, sf::Vector2f anchor);
    
    void SetConstraintIterations(int iterations) { m_constraintIterations = iterations; }
    
    void Step(float dt);
};

#endif //PHYSICSENGINE_PHYSICSWORLD_H
