#ifndef PHYSICSENGINE_COUNTERPANEL_H
#define PHYSICSENGINE_COUNTERPANEL_H

#include <SFML/Graphics.hpp>
#include <sstream>

class CounterPanel : public sf::Drawable {
private:
    sf::Text m_text;
    sf::RectangleShape m_box;
    float m_windowWidth;
    
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override {
        target.draw(m_box, states);
        target.draw(m_text, states);
    }

public:
    CounterPanel(const sf::Font& font, float windowWidth) 
        : m_text(font, "", 14), m_windowWidth(windowWidth) {
        m_text.setFillColor(sf::Color::White);
        
        m_box.setFillColor(sf::Color(20, 25, 40, 220));
        m_box.setOutlineColor(sf::Color(80, 90, 120));
        m_box.setOutlineThickness(1.f);
    }
    
    void update(size_t ballCount, size_t floorCount) {
        size_t total = ballCount + floorCount;
        
        std::ostringstream ss;
        ss << "Balls: " << ballCount << "\n";
        ss << "Floors: " << floorCount << "\n";
        ss << "Total: " << total;
        
        m_text.setString(ss.str());
        
        sf::FloatRect bounds = m_text.getLocalBounds();
        float boxWidth = bounds.size.x + 20.f;
        float boxHeight = bounds.size.y + 20.f;
        
        m_box.setSize({boxWidth, boxHeight});
        m_box.setPosition({m_windowWidth - boxWidth - 5.f, 5.f});
        m_text.setPosition({m_windowWidth - boxWidth + 5.f, 10.f});
    }
};

#endif //PHYSICSENGINE_COUNTERPANEL_H

