#ifndef PHYSICSENGINE_INFOPANEL_H
#define PHYSICSENGINE_INFOPANEL_H

#include <SFML/Graphics.hpp>
#include <sstream>
#include <iomanip>
#include <cmath>
#include "../Object.h"

class InfoPanel : public sf::Drawable {
private:
    sf::Text m_text;
    sf::RectangleShape m_box;
    bool m_visible = false;
    
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override {
        if (m_visible) {
            target.draw(m_box, states);
            target.draw(m_text, states);
        }
    }

public:
    InfoPanel(const sf::Font& font) : m_text(font, "", 14) {
        m_text.setFillColor(sf::Color::White);
        m_text.setPosition({10.f, 10.f});
        
        m_box.setFillColor(sf::Color(20, 25, 40, 220));
        m_box.setOutlineColor(sf::Color(80, 90, 120));
        m_box.setOutlineThickness(1.f);
        m_box.setPosition({5.f, 5.f});
    }
    
    void update(const Object* obj, float radius) {
        if (!obj) {
            m_visible = false;
            return;
        }
        
        m_visible = true;
        
        sf::Vector2f vel = obj->position - obj->oldPosition;
        float speed = std::sqrt(vel.x * vel.x + vel.y * vel.y) * 60.f;
        
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(1);
        ss << "Position: (" << obj->position.x << ", " << obj->position.y << ")\n";
        ss << "Velocity: (" << vel.x * 60.f << ", " << vel.y * 60.f << ")\n";
        ss << "Speed: " << speed << " px/s\n";
        ss << "Mass: " << obj->mass << "\n";
        ss << "Radius: " << radius;
        
        m_text.setString(ss.str());
        
        sf::FloatRect textBounds = m_text.getLocalBounds();
        m_box.setSize({textBounds.size.x + 20.f, textBounds.size.y + 20.f});
    }
    
    void hide() { m_visible = false; }
    bool isVisible() const { return m_visible; }
};

#endif //PHYSICSENGINE_INFOPANEL_H

