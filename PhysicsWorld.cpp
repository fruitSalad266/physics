#include "PhysicsWorld.h"
#include <cmath>
#include <algorithm>

PhysicsWorld::PhysicsWorld() {
    m_gravity = {0.f, 1000.f};
}

void PhysicsWorld::AddObject(Object* object) {
    m_objects.push_back(object);
}

void PhysicsWorld::RemoveObject(Object *object) {
    m_objects.erase(std::remove(m_objects.begin(), m_objects.end(), object), m_objects.end());
}

// ============ CONSTRAINT MANAGEMENT ============

Constraint* PhysicsWorld::AddConstraint(std::unique_ptr<Constraint> constraint) {
    m_constraints.push_back(std::move(constraint)); //cast so that we can move the constraint
    return m_constraints.back().get();
}

void PhysicsWorld::RemoveConstraint(Constraint* constraint) {
    m_constraints.erase(
        std::remove_if(m_constraints.begin(), m_constraints.end(),
            [constraint](const std::unique_ptr<Constraint>& c) { 
                return c.get() == constraint; 
            }),
        m_constraints.end()
    );
}

DistanceConstraint* PhysicsWorld::AddDistanceConstraint(Object* a, Object* b, float length) {
    std::unique_ptr<DistanceConstraint> c;
    if (length < 0) {
        c = std::make_unique<DistanceConstraint>(a, b);  // Auto-calculate length
    } else {
        c = std::make_unique<DistanceConstraint>(a, b, length, 1.0f);  // Explicit length + stiffness
    }
    auto* ptr = c.get();
    m_constraints.push_back(std::move(c));
    return ptr;
}

SpringConstraint* PhysicsWorld::AddSpringConstraint(Object* a, Object* b, float stiffness, float damping) {
    sf::Vector2f diff = b->position - a->position;
    float length = std::sqrt(diff.x * diff.x + diff.y * diff.y);
    auto c = std::make_unique<SpringConstraint>(a, b, length, stiffness, damping);
    auto* ptr = c.get();
    m_constraints.push_back(std::move(c));
    return ptr;
}

PinConstraint* PhysicsWorld::AddPinConstraint(Object* obj, sf::Vector2f anchor) {
    auto c = std::make_unique<PinConstraint>(obj, anchor);
    auto* ptr = c.get();
    m_constraints.push_back(std::move(c));
    return ptr;
}

//Uses Gauss-seidel relaxation (i.e. solve each constraint in each iteration)
void PhysicsWorld::SolveConstraints() {
    for (int i = 0; i < m_constraintIterations; ++i) {
        for (auto& constraint : m_constraints) {
            constraint->Solve();
        }
    }
}

//forces -> integration -> constraints -> collisions
void PhysicsWorld::Step(float dt) {
    //1. Gravity
    for (Object* obj : m_objects) {
        if (obj->isStatic) continue;
        obj->acceleration = m_gravity;
    }

    // 2. Verlet integration
    for (Object* obj : m_objects) {
        if (obj->isStatic) continue;

        sf::Vector2f temp = obj->position;
        
        // Verlet: newPos = pos + (pos - oldPos) + accel * dt^2
        sf::Vector2f velocity = obj->position - obj->oldPosition;
        obj->position = obj->position + velocity + obj->acceleration * dt * dt;
        obj->oldPosition = temp;
    }

    // 3. Solve constraints (iteratively for stability)
    SolveConstraints();

    // 4. Collision detection and resolution
    for (size_t i = 0; i < m_objects.size(); ++i) {
        for (size_t j = i + 1; j < m_objects.size(); ++j) {
            Object* objA = m_objects[i];
            Object* objB = m_objects[j];

            if (objA->isStatic && objB->isStatic) continue;
            if (!objA->collider || !objB->collider) continue;

            ResolveCollision(objA, objB);
        }
    }
}

//helper function for which collision resolution function to call
void PhysicsWorld::ResolveCollision(Object* objA, Object* objB) {
    ColliderType typeA = objA->collider->type;
    ColliderType typeB = objB->collider->type;

    if (typeA == ColliderType::Circle && typeB == ColliderType::Circle) {
        ResolveCircleCircle(objA, objB);
    } 
    else if (typeA == ColliderType::Circle && typeB == ColliderType::AABB) {
        ResolveCircleAABB(objA, objB);
    } 
    else if (typeA == ColliderType::AABB && typeB == ColliderType::Circle) {
        ResolveCircleAABB(objB, objA);  // Swap order so circle is first
    }
    else if (typeA == ColliderType::AABB && typeB == ColliderType::AABB) {
        ResolveAABBAABB(objA, objB);
    }
}

void PhysicsWorld::ResolveCircleCircle(Object* objA, Object* objB) {
    auto* circleA = objA->GetCircleCollider();
    auto* circleB = objB->GetCircleCollider();
    if (!circleA || !circleB) return;

    sf::Vector2f diff = objB->position - objA->position;
    float distSq = diff.x * diff.x + diff.y * diff.y;
    float radiusSum = circleA->radius + circleB->radius;

    if (distSq < radiusSum * radiusSum) {
        float distance = std::sqrt(distSq);
        sf::Vector2f normal; //normal vector from center A to B
        float penetration;
        
        if (distance < 0.0001f) {
            normal = {1.f, 0.f};
            penetration = radiusSum;
        } else {
            normal = diff / distance;
            penetration = radiusSum - distance;
        }

        // Store velocities BEFORE any changes
        sf::Vector2f velA = objA->position - objA->oldPosition;
        sf::Vector2f velB = objB->position - objB->oldPosition;

        // Positional correction - move BOTH to preserve velocity
        sf::Vector2f correction = normal * (penetration * 0.5f);
        if (!objA->isStatic) {
            objA->position -= correction;
            objA->oldPosition -= correction;
        }
        if (!objB->isStatic) {
            objB->position += correction;
            objB->oldPosition += correction;
        }

        // Elastic collision response
        float bounceStrength = 0.5f;
        sf::Vector2f relVel = velA - velB;
        float relVelAlongNormal = relVel.x * normal.x + relVel.y * normal.y;
        
        // Only apply bounce if objects are approaching
        if (relVelAlongNormal > 0) {
            sf::Vector2f impulse = normal * (relVelAlongNormal * (1.f + bounceStrength) * 0.5f);
            
            // Adjust oldPosition to change velocity (not position)
            if (!objA->isStatic) objA->oldPosition += impulse;
            if (!objB->isStatic) objB->oldPosition -= impulse;
        }
    }
}

void PhysicsWorld::ResolveCircleAABB(Object* circle, Object* box) {
    auto* circleCollider = circle->GetCircleCollider();
    auto* boxCollider = box->GetAABBCollider();
    if (!circleCollider || !boxCollider) return;

    sf::Vector2f boxMin = box->position - boxCollider->halfExtents;
    sf::Vector2f boxMax = box->position + boxCollider->halfExtents;

    sf::Vector2f closestPoint;
    closestPoint.x = std::clamp(circle->position.x, boxMin.x, boxMax.x);
    closestPoint.y = std::clamp(circle->position.y, boxMin.y, boxMax.y);

    sf::Vector2f diff = circle->position - closestPoint;
    float distSq = diff.x * diff.x + diff.y * diff.y;
    float radius = circleCollider->radius;

    if (distSq < radius * radius) {
        float distance = std::sqrt(distSq);
        sf::Vector2f normal;
        float penetration;
        
        if (distance < 0.0001f) {
            float overlapX = boxCollider->halfExtents.x - std::abs(circle->position.x - box->position.x);
            float overlapY = boxCollider->halfExtents.y - std::abs(circle->position.y - box->position.y);
            
            if (overlapX < overlapY) {
                normal = {(circle->position.x < box->position.x) ? -1.f : 1.f, 0.f};
                penetration = overlapX + radius;
            } else {
                normal = {0.f, (circle->position.y < box->position.y) ? -1.f : 1.f};
                penetration = overlapY + radius;
            }
        } else {
            normal = diff / distance;
            penetration = radius - distance;
        }

        // Store velocity BEFORE any changes
        sf::Vector2f vel = circle->position - circle->oldPosition;
        float velAlongNormal = vel.x * normal.x + vel.y * normal.y;

        // Positional correction - move BOTH to preserve velocity
        sf::Vector2f correction = normal * penetration;
        if (!circle->isStatic) {
            circle->position += correction;
            circle->oldPosition += correction;
        }
        if (!box->isStatic) {
            box->position -= correction;
            box->oldPosition -= correction;
        }

        // Apply bounce if moving toward the surface
        if (!circle->isStatic && velAlongNormal < 0) {
            sf::Vector2f tangent = {-normal.y, normal.x};
            float velAlongTangent = vel.x * tangent.x + vel.y * tangent.y;
            
            // Reflect: reverse normal component, apply friction to tangent
            float newNormalVel = -velAlongNormal * circle->bounciness;
            float newTangentVel = velAlongTangent * 0.98f;
            
            // Adjust oldPosition to create new velocity
            // velocity change = newVel - oldVel
            // oldPosition change = -(velocity change) = oldVel - newVel
            sf::Vector2f oldNormalComp = normal * velAlongNormal;
            sf::Vector2f oldTangentComp = tangent * velAlongTangent;
            sf::Vector2f newNormalComp = normal * newNormalVel;
            sf::Vector2f newTangentComp = tangent * newTangentVel;
            
            sf::Vector2f velChange = (newNormalComp + newTangentComp) - (oldNormalComp + oldTangentComp);
            circle->oldPosition -= velChange;
        }
    }
}

void PhysicsWorld::ResolveAABBAABB(Object* objA, Object* objB) {
    auto* boxA = objA->GetAABBCollider();
    auto* boxB = objB->GetAABBCollider();
    if (!boxA || !boxB) return;

    sf::Vector2f aMin = objA->position - boxA->halfExtents;
    sf::Vector2f aMax = objA->position + boxA->halfExtents;
    sf::Vector2f bMin = objB->position - boxB->halfExtents;
    sf::Vector2f bMax = objB->position + boxB->halfExtents;

    if (aMax.x < bMin.x || aMin.x > bMax.x) return; 
    if (aMax.y < bMin.y || aMin.y > bMax.y) return;

    float overlapX = std::min(aMax.x - bMin.x, bMax.x - aMin.x);
    float overlapY = std::min(aMax.y - bMin.y, bMax.y - aMin.y);

    sf::Vector2f normal;
    float penetration;

    if (overlapX < overlapY) {
        penetration = overlapX;
        normal = (objA->position.x < objB->position.x) ? sf::Vector2f{-1.f, 0.f} : sf::Vector2f{1.f, 0.f};
    } else {
        penetration = overlapY;
        normal = (objA->position.y < objB->position.y) ? sf::Vector2f{0.f, -1.f} : sf::Vector2f{0.f, 1.f};
    }

    // Store velocities BEFORE any changes
    sf::Vector2f velA = objA->position - objA->oldPosition;
    sf::Vector2f velB = objB->position - objB->oldPosition;

    // Positional correction - move BOTH to preserve velocity
    sf::Vector2f correction = normal * (penetration * 0.5f);
    if (!objA->isStatic) {
        objA->position += correction;
        objA->oldPosition += correction;
    }
    if (!objB->isStatic) {
        objB->position -= correction;
        objB->oldPosition -= correction;
    }

    // Bounce response
    float bounceStrength = 0.5f;
    sf::Vector2f relVel = velA - velB;
    float relVelAlongNormal = relVel.x * normal.x + relVel.y * normal.y;
    
    if (relVelAlongNormal < 0) {
        sf::Vector2f impulse = normal * (-relVelAlongNormal * (1.f + bounceStrength) * 0.5f);
        
        if (!objA->isStatic) objA->oldPosition -= impulse;
        if (!objB->isStatic) objB->oldPosition += impulse;
    }
}
