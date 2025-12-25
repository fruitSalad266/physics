//Tests for Distance, Pin constraints, Spring constraints, Static objects.

#include <gtest/gtest.h>
#include "Object.h"
#include "Constraint.h"
#include "PhysicsWorld.h"
#include <cmath>

// helper to calculate distance
float Distance(const Object& a, const Object& b) {
    sf::Vector2f diff = b.position - a.position;
    return std::sqrt(diff.x * diff.x + diff.y * diff.y);
}

//Distance

class DistanceConstraintTest : public ::testing::Test {
protected:
    void SetUp() override {
        objA.position = {0.f, 0.f};
        objB.position = {100.f, 0.f};
        objA.InitVerlet();
        objB.InitVerlet();
    }
    
    Object objA, objB;
};

TEST_F(DistanceConstraintTest, MaintainsLengthWhenStretched) {
    DistanceConstraint constraint(&objA, &objB);  // rest length = 100
    
    // Stretch the constraint
    objB.position = {200.f, 0.f};
    
    // Solve multiple times for stability
    for (int i = 0; i < 10; ++i) {
        constraint.Solve();
    }
    
    float dist = Distance(objA, objB);
    EXPECT_NEAR(dist, 100.f, 0.1f);
}

TEST_F(DistanceConstraintTest, MaintainsLengthWhenCompressed) {
    DistanceConstraint constraint(&objA, &objB);  // rest length = 100
    
    // Compress the constraint
    objB.position = {30.f, 0.f};
    
    for (int i = 0; i < 10; ++i) {
        constraint.Solve();
    }
    
    float dist = Distance(objA, objB);
    EXPECT_NEAR(dist, 100.f, 0.1f);
}

TEST_F(DistanceConstraintTest, StaticObjectDoesNotMove) {
    objA.isStatic = true;
    sf::Vector2f originalPosA = objA.position;
    
    DistanceConstraint constraint(&objA, &objB);
    
    // Perturb B
    objB.position = {200.f, 0.f};
    
    for (int i = 0; i < 10; ++i) {
        constraint.Solve();
    }
    
    EXPECT_FLOAT_EQ(objA.position.x, originalPosA.x);
    EXPECT_FLOAT_EQ(objA.position.y, originalPosA.y);
    
    // Correct dist
    float dist = Distance(objA, objB);
    EXPECT_NEAR(dist, 100.f, 0.1f);
}

TEST_F(DistanceConstraintTest, AutoCalculatesRestLength) {
    objB.position = {75.f, 0.f};  // Distance is 75
    
    DistanceConstraint constraint(&objA, &objB);  // Should auto-calc to 75
    
    // Move B away
    objB.position = {150.f, 0.f};
    
    for (int i = 0; i < 10; ++i) {
        constraint.Solve();
    }
    
    // Should return to 75, not 100
    float dist = Distance(objA, objB);
    EXPECT_NEAR(dist, 75.f, 0.1f);
}

// ============ PIN CONSTRAINT TESTS ============

class PinConstraintTest : public ::testing::Test {
protected:
    void SetUp() override {
        obj.position = {100.f, 100.f};
        obj.InitVerlet();
    }
    
    Object obj;
};

TEST_F(PinConstraintTest, LocksPositionToAnchor) {
    sf::Vector2f anchor = {50.f, 50.f};
    PinConstraint pin(&obj, anchor);
    
    pin.Solve();
    
    EXPECT_FLOAT_EQ(obj.position.x, anchor.x);
    EXPECT_FLOAT_EQ(obj.position.y, anchor.y);
}

TEST_F(PinConstraintTest, MaintainsPositionAfterMovement) {
    sf::Vector2f anchor = {50.f, 50.f};
    PinConstraint pin(&obj, anchor);
    
    // Try to move the object
    obj.position = {200.f, 300.f};
    
    pin.Solve();
    
    // Should snap back to anchor
    EXPECT_FLOAT_EQ(obj.position.x, anchor.x);
    EXPECT_FLOAT_EQ(obj.position.y, anchor.y);
}

TEST_F(PinConstraintTest, AnchorCanBeUpdated) {
    PinConstraint pin(&obj, {0.f, 0.f});
    
    pin.SetAnchor({999.f, 888.f});
    pin.Solve();
    
    EXPECT_FLOAT_EQ(obj.position.x, 999.f);
    EXPECT_FLOAT_EQ(obj.position.y, 888.f);
}

// ============ SPRING CONSTRAINT TESTS ============

class SpringConstraintTest : public ::testing::Test {
protected:
    void SetUp() override {
        objA.position = {0.f, 0.f};
        objB.position = {100.f, 0.f};
        objA.InitVerlet();
        objB.InitVerlet();
    }
    
    Object objA, objB;
};

TEST_F(SpringConstraintTest, PullsTowardRestLength) {
    float restLength = 100.f;
    SpringConstraint spring(&objA, &objB, restLength, 0.5f, 0.0f);
    
    // Stretch the spring
    objB.position = {200.f, 0.f};
    float initialDist = Distance(objA, objB);
    
    spring.Solve();
    
    float newDist = Distance(objA, objB);
    
    // Should have moved closer to rest length
    EXPECT_LT(newDist, initialDist);
}

TEST_F(SpringConstraintTest, SofterThanDistanceConstraint) {
    // Spring with low stiffness should correct less per solve
    float restLength = 100.f;
    SpringConstraint spring(&objA, &objB, restLength, 0.1f, 0.0f);
    
    objB.position = {200.f, 0.f};
    spring.Solve();
    
    float springDist = Distance(objA, objB);
    
    // Reset
    objA.position = {0.f, 0.f};
    objB.position = {200.f, 0.f};
    
    DistanceConstraint rigid(&objA, &objB, 100.f, 1.0f);
    rigid.Solve();
    
    float rigidDist = Distance(objA, objB);
    
    // Spring should be further from rest length (corrected less)
    EXPECT_GT(std::abs(springDist - restLength), std::abs(rigidDist - restLength));
}

// ============ INTEGRATION TESTS ============

TEST(PhysicsWorldConstraints, ConstraintsAreSolved) {
    PhysicsWorld world;
    
    Object pivot, mass;
    pivot.position = {400.f, 100.f};
    pivot.isStatic = true;
    pivot.InitVerlet();
    
    mass.position = {400.f, 200.f};
    mass.InitVerlet();
    
    world.AddObject(&pivot);
    world.AddObject(&mass);
    
    auto* constraint = world.AddDistanceConstraint(&pivot, &mass);
    
    // Perturb the mass
    mass.position = {500.f, 300.f};
    
    // Step the world (which should solve constraints)
    world.Step(1.f / 60.f);
    
    // Distance should be approximately maintained
    float dist = Distance(pivot, mass);
    EXPECT_NEAR(dist, 100.f, 5.f);  // Allow some tolerance due to gravity
}

