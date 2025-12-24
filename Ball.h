#ifndef PHYSICSENGINE_BALL_H
#define PHYSICSENGINE_BALL_H

#include <SFML/Graphics.hpp>
#include <cstdint>
#include "Object.h"

struct Ball {
    Object physics;
    sf::CircleShape shape;

    Ball(float x, float y, float r, sf::Color c) {
        physics.position = {x, y};
        physics.InitVerlet();  // Initialize oldPosition = position (starts at rest)
        physics.SetCircleCollider(r);
        
        // Soften color
        sf::Color softened(
            static_cast<std::uint8_t>(c.r * 0.7f + 40),
            static_cast<std::uint8_t>(c.g * 0.7f + 40),
            static_cast<std::uint8_t>(c.b * 0.7f + 40),
            c.a
        );
        
        sf::Color outline = sf::Color::White;
        
        shape.setRadius(r);
        shape.setOrigin({r, r});
        shape.setFillColor(softened);
        shape.setOutlineThickness(3.f);
        shape.setOutlineColor(outline);
        shape.setPosition(physics.position);
    }

    void render(sf::RenderWindow& w) {
        shape.setPosition(physics.position);
        w.draw(shape);
    }
    
    float GetRadius() const {
        if (auto* circle = physics.GetCircleCollider()) {
            return circle->radius; //if there is a circle collider
        }
        return 0.f; //if there is no circle collider, return 0
    }
};

#endif //PHYSICSENGINE_BALL_H

