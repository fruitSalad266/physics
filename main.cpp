#include <SFML/Graphics.hpp>
#include "PhysicsWorld.h"
#include "Ball.h"
#include "Spring.h"
#include "Grid.h"
#include "UI/InfoPanel.h"
#include "UI/CounterPanel.h"

#include <vector>
#include <iostream>

int main() {
    const int width = 800;
    const int height = 600;

    sf::RenderWindow window(sf::VideoMode({width, height}), "Matt is epic");
    window.setFramerateLimit(60);

    PhysicsWorld world;
    std::vector<std::unique_ptr<Ball>> myBalls; //only pointers get moved
    //so that we can allocate each Ball at a stable address
    //Only unique_ptr are moved, but the Balls aren't moved
    //i.e., don't read from garbage memory

    Object floorP;
    floorP.position = {400.f, 600.f};
    floorP.isStatic = true;
    floorP.InitVerlet();
    floorP.SetAABBCollider(800.f, 100.f);
    world.AddObject(&floorP);

    sf::RectangleShape floor({800, 100});
    floor.setPosition({0.f, 550.f});
    floor.setFillColor(sf::Color(60, 60, 70));

    const sf::Color bgColor(25, 35, 60);        
    Grid grid(width, height, 50, sf::Color(55, 65, 90));

    // Load font for UI
    sf::Font font;
    bool fontLoaded = font.openFromFile("/System/Library/Fonts/SFNSMono.ttf");
    if (!fontLoaded) {
        fontLoaded = font.openFromFile("/System/Library/Fonts/Menlo.ttc");
    }
    
    // UI panels
    InfoPanel infoPanel(font);
    CounterPanel counterPanel(font, static_cast<float>(width));
    
    int floorCount = 1;
    Ball* hoveredBall = nullptr;
    
    auto anchorBall = std::make_unique<Ball>(600.f, 50.f, 10.f, sf::Color(100, 200, 100));
    auto swingBall = std::make_unique<Ball>(650.f, 150.f, 18.f, sf::Color(220, 180, 80));
    
    world.AddObject(&anchorBall->physics);
    world.AddObject(&swingBall->physics);
    
    world.AddPinConstraint(&anchorBall->physics, anchorBall->physics.position);
    
    // Connect with spring
    auto* springConstraint = world.AddSpringConstraint(
        &anchorBall->physics, &swingBall->physics, 0.3f, 0.05f);
    
    Spring spring(&anchorBall->physics, &swingBall->physics, springConstraint,
                  sf::Color(255, 200, 100), 2.f, 12);
    
    myBalls.push_back(std::move(anchorBall));
    myBalls.push_back(std::move(swingBall));

    sf::Clock clock;

    while (window.isOpen()) {
        while (const auto event = window.pollEvent()) {
            if (event->is<sf::Event::Closed>()) window.close();

            if (event->is<sf::Event::MouseButtonPressed>()) {
                const auto* mouse = event->getIf<sf::Event::MouseButtonPressed>();
                if (mouse->button == sf::Mouse::Button::Left) {
                    auto ball = std::make_unique<Ball>(static_cast<float>(mouse->position.x), static_cast<float>(mouse->position.y), 25.f, sf::Color(220, 120, 100));
                    world.AddObject(&ball->physics);
                    myBalls.push_back(std::move(ball));
                }
            }
        }

        const float dt = clock.restart().asSeconds();

        for(int i=0; i<8; i++) {
            world.Step(dt / 8.f);
        }
        
        // Remove balls that are off-screen
        for (auto it = myBalls.begin(); it != myBalls.end(); ) {
            float r = (*it)->GetRadius();
            sf::Vector2f pos = (*it)->physics.position;
            
            bool offScreen = (pos.x + r < 0) || (pos.x - r > width) ||
                             (pos.y + r < 0) || (pos.y - r > height + 200);  // Extra margin below
            
            if (offScreen) {
                world.RemoveObject(&(*it)->physics);
                it = myBalls.erase(it);
            } else {
                ++it;
            }
        }
        
        // Check for hover
        sf::Vector2i mousePos = sf::Mouse::getPosition(window);
        sf::Vector2f mousePosF(static_cast<float>(mousePos.x), static_cast<float>(mousePos.y));
        hoveredBall = nullptr;
        
        for (auto& ball : myBalls) {
            sf::Vector2f diff = ball->physics.position - mousePosF;
            float distSq = diff.x * diff.x + diff.y * diff.y;
            float radius = ball->GetRadius();
            if (distSq < radius * radius) {
                hoveredBall = ball.get();
                break;
            }
        }
        
        // Update UI
        if (hoveredBall) {
            infoPanel.update(&hoveredBall->physics, hoveredBall->GetRadius());
        } else {
            infoPanel.hide();
        }
        counterPanel.update(myBalls.size(), floorCount);
        
        // Render
        window.clear(bgColor);
        window.draw(grid);
        window.draw(floor);

        for (auto& ball : myBalls) {
            ball->render(window);
        }
        
        spring.render(window);
        
        if (fontLoaded) {
            window.draw(counterPanel);
            window.draw(infoPanel);
        }

        window.display();
    }

    return 0;
}
