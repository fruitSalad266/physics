//Vertlet tests

#include <gtest/gtest.h>
#include "Object.h"
#include "PhysicsWorld.h"
#include <cmath>

class VerletTest : public ::testing::Test {
protected:
    void SetUp() override {
        obj.position = {100.f, 100.f};
        obj.InitVerlet();
        obj.mass = 1.0f;
    }
    
    Object obj;
    const float dt = 1.f / 60.f;
    const sf::Vector2f gravity = {0.f, 1000.f};
    
    // Helper: perform one Verlet step
    void VerletStep(Object& o, sf::Vector2f accel) {
        sf::Vector2f temp = o.position;
        sf::Vector2f velocity = o.position - o.oldPosition;
        o.position = o.position + velocity + accel * dt * dt;
        o.oldPosition = temp;
    }
};

TEST_F(VerletTest, ObjectStartsAtRest) {
    // After InitVerlet, velocity should be zero
    sf::Vector2f vel = obj.position - obj.oldPosition;
    EXPECT_FLOAT_EQ(vel.x, 0.f);
    EXPECT_FLOAT_EQ(vel.y, 0.f);
}

TEST_F(VerletTest, GravityAcceleratesObject) {
    float initialY = obj.position.y;
    
    for (int i = 0; i < 60; ++i) { 
        VerletStep(obj, gravity);
    }
    
    // Object should have fallen somewhat
    EXPECT_GT(obj.position.y, initialY + 100.f);
}

TEST_F(VerletTest, VelocityAccumulates) {
    // Each frame, velocity should increase due to gravity
    float prevVelY = 0.f;
    
    for (int i = 0; i < 10; ++i) {
        VerletStep(obj, gravity);
        float velY = obj.position.y - obj.oldPosition.y;
        EXPECT_GT(velY, prevVelY) << "Frame " << i << ": velocity should increase";
        prevVelY = velY;
    }
}

TEST_F(VerletTest, NoAccelerationMaintainsVelocity) {
    // Give object initial velocity
    obj.oldPosition = obj.position - sf::Vector2f{5.f, 0.f};
    
    float initialVelX = obj.position.x - obj.oldPosition.x;
    
    // Step with no acceleration
    for (int i = 0; i < 10; ++i) {
        VerletStep(obj, {0.f, 0.f});
    }
    
    float finalVelX = obj.position.x - obj.oldPosition.x;
    
    // Velocity should remain (kind of)constant
    EXPECT_NEAR(finalVelX, initialVelX, 0.001f);
}

TEST_F(VerletTest, SetVelocityWorks) {
    sf::Vector2f targetVel = {10.f, -5.f};
    obj.SetVelocity(targetVel, dt);
    
    sf::Vector2f actualVel = obj.GetVelocity(dt);
    
    EXPECT_NEAR(actualVel.x, targetVel.x, 0.001f);
    EXPECT_NEAR(actualVel.y, targetVel.y, 0.001f);
}

