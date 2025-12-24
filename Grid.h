#ifndef PHYSICSENGINE_GRID_H
#define PHYSICSENGINE_GRID_H

#include <SFML/Graphics.hpp>

class Grid : public sf::Drawable {
private:
    sf::VertexArray m_lines;
    
    void draw(sf::RenderTarget& target, sf::RenderStates states) const override {
        target.draw(m_lines, states);
    }

public:
    Grid(int width, int height, int spacing, sf::Color color) {
        m_lines.setPrimitiveType(sf::PrimitiveType::Lines);
        
        // Vertical lines
        for (int x = 0; x <= width; x += spacing) {
            m_lines.append(sf::Vertex{{static_cast<float>(x), 0.f}, color});
            m_lines.append(sf::Vertex{{static_cast<float>(x), static_cast<float>(height)}, color});
        }
        
        // Horizontal lines
        for (int y = 0; y <= height; y += spacing) {
            m_lines.append(sf::Vertex{{0.f, static_cast<float>(y)}, color});
            m_lines.append(sf::Vertex{{static_cast<float>(width), static_cast<float>(y)}, color});
        }
    }
};

#endif //PHYSICSENGINE_GRID_H

