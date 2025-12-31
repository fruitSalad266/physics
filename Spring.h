#ifndef PHYSICSENGINE_SPRING_H
#define PHYSICSENGINE_SPRING_H

#include <SFML/Graphics.hpp>
#include <cmath>
#include "Object.h"
#include "Constraint.h"

//just a zigzag for now lol
struct Spring {
    Object* objA;
    Object* objB;
    SpringConstraint* constraint;  // Owned by PhysicsWorld
    
    sf::Color color;
    float thickness;
    int coils;  // no. of segments
    
    Spring(Object* a, Object* b, SpringConstraint* c, 
           sf::Color col = sf::Color(180, 180, 200), 
           float thick = 2.f, 
           int numCoils = 8)
        : objA(a), objB(b), constraint(c), 
          color(col), thickness(thick), coils(numCoils) {}
    
    void render(sf::RenderWindow& w) {
        sf::Vector2f start = objA->position;
        sf::Vector2f end = objB->position;
        
        sf::Vector2f diff = end - start;
        float length = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        
        if (length < 0.001f) return;

        sf::Vector2f dir = diff / length;
        sf::Vector2f perp = {-dir.y, dir.x};

        float restLength = constraint ? constraint->restLength : length;
        float stretchRatio = length / restLength;
        float amplitude = 8.f / std::max(stretchRatio, 0.5f);  // Wider when compressed

        sf::VertexArray lines(sf::PrimitiveType::LineStrip, coils + 3);
        
        // Start at objA
        lines[0].position = start;
        lines[0].color = color;

        lines[1].position = start + dir * (length * 0.1f);
        lines[1].color = color;
        
        // Zigzag middle section
        float segmentLength = (length * 0.8f) / static_cast<float>(coils);
        for (int i = 0; i < coils; ++i) {
            float t = 0.1f + (static_cast<float>(i) + 0.5f) * 0.8f / static_cast<float>(coils);
            float side = (i % 2 == 0) ? 1.f : -1.f;
            
            sf::Vector2f pos = start + dir * (length * t) + perp * (amplitude * side);
            lines[i + 2].position = pos;
            lines[i + 2].color = color;
        }
        
        //ObjB aka end
        lines[coils + 2].position = end;
        lines[coils + 2].color = color;
        
        w.draw(lines);
    }

    float GetStretch() const {
        if (!constraint) return 1.f;
        
        sf::Vector2f diff = objB->position - objA->position;
        float currentLength = std::sqrt(diff.x * diff.x + diff.y * diff.y);
        return currentLength / constraint->restLength;
    }
    
    float GetRestLength() const {
        return constraint ? constraint->restLength : 0.f;
    }
    
    float GetStiffness() const {
        return constraint ? constraint->stiffness : 0.f;
    }
    
    float GetDamping() const {
        return constraint ? constraint->damping : 0.f;
    }
};

#endif //PHYSICSENGINE_SPRING_H

