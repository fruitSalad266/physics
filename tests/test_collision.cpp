//Between AABB and Circle, etc.

#include <gtest/gtest.h>
#include "Object.h"
#include "Collider.h"
#include "PhysicsWorld.h"
#include <cmath>

float Distance(const sf::Vector2f& a, const sf::Vector2f& b) {
    sf::Vector2f diff = b - a;
    return std::sqrt(diff.x * diff.x + diff.y * diff.y);
}


class CircleCollisionTest : public ::testing::Test {
protected:
    void SetUp() override {
        world = std::make_unique<PhysicsWorld>();
        
        objA.position = {100.f, 100.f};
        objA.InitVerlet();
        objA.SetCircleCollider(25.f);
        
        objB.position = {200.f, 100.f};
        objB.InitVerlet();
        objB.SetCircleCollider(25.f);
        
        world->AddObject(&objA);
        world->AddObject(&objB);
    }
    
    std::unique_ptr<PhysicsWorld> world;
    Object objA, objB;
};

TEST_F(CircleCollisionTest, NoCollisionWhenApart) {
    sf::Vector2f originalA = objA.position;
    sf::Vector2f originalB = objB.position;
    
    // Objects are 100 apart, radii sum to 50 - no collision
    world->Step(1.f / 60.f);
    
    //should be in same x
    EXPECT_FLOAT_EQ(objA.position.x, originalA.x);
    EXPECT_FLOAT_EQ(objB.position.x, originalB.x);
}

TEST_F(CircleCollisionTest, ResolvesOverlap) {
    // Move objects to overlap
    objA.position = {100.f, 100.f};
    objB.position = {130.f, 100.f};  // 30 apart, but radii sum to 50
    objA.oldPosition = objA.position;
    objB.oldPosition = objB.position;
    
    world->Step(1.f / 60.f);
    
    float dist = Distance(objA.position, objB.position);
    EXPECT_GE(dist, 49.f);  
}

TEST_F(CircleCollisionTest, CollisionPushesApart) {
    // Place objects overlapping
    objA.position = {100.f, 100.f};
    objB.position = {110.f, 100.f};  // Heavy overlap
    objA.oldPosition = objA.position;
    objB.oldPosition = objB.position;
    
    float initialDist = Distance(objA.position, objB.position);
    
    world->Step(1.f / 60.f);
    
    float finalDist = Distance(objA.position, objB.position);
    
    EXPECT_GT(finalDist, initialDist);
}

// ============ CIRCLE-AABB COLLISION TESTS ============

class CircleAABBCollisionTest : public ::testing::Test {
protected:
    void SetUp() override {
        world = std::make_unique<PhysicsWorld>();
        
        // Circle
        ball.position = {100.f, 100.f};
        ball.InitVerlet();
        ball.SetCircleCollider(20.f);
        
        // Static floor
        floor.position = {100.f, 200.f};
        floor.isStatic = true;
        floor.InitVerlet();
        floor.SetAABBCollider(200.f, 50.f);  // 200 wide, 50 tall
        
        world->AddObject(&ball);
        world->AddObject(&floor);
    }
    
    std::unique_ptr<PhysicsWorld> world;
    Object ball, floor;
};

TEST_F(CircleAABBCollisionTest, BallBouncesOffFloor) {
    // Drop ball onto floor
    ball.position = {100.f, 170.f};  // Just above floor (floor top is at 175)
    ball.oldPosition = {100.f, 160.f};  // Moving down
    
    world->Step(1.f / 60.f);
    
    // Ball should be pushed up and not inside floor
    // Floor extends from y=175 to y=225 (position 200, halfExtent 25)
    float floorTop = floor.position.y - 25.f;
    float ballBottom = ball.position.y + 20.f;
    
    EXPECT_LE(ballBottom, floorTop + 1.f);  // Ball should not penetrate floor
}

TEST_F(CircleAABBCollisionTest, StaticFloorDoesNotMove) {
    sf::Vector2f originalPos = floor.position;
    
    // Drop ball onto floor
    ball.position = {100.f, 170.f};
    ball.oldPosition = {100.f, 150.f};
    
    for (int i = 0; i < 10; ++i) {
        world->Step(1.f / 60.f);
    }
    
    EXPECT_FLOAT_EQ(floor.position.x, originalPos.x);
    EXPECT_FLOAT_EQ(floor.position.y, originalPos.y);
}

// ============ COLLIDER CREATION TESTS ============

TEST(ColliderTest, CircleColliderHasCorrectRadius) {
    Object obj;
    obj.SetCircleCollider(42.f);
    
    auto* circle = obj.GetCircleCollider();
    ASSERT_NE(circle, nullptr);
    EXPECT_FLOAT_EQ(circle->radius, 42.f);
}

TEST(ColliderTest, AABBColliderHasCorrectExtents) {
    Object obj;
    obj.SetAABBCollider(100.f, 50.f);
    
    auto* aabb = obj.GetAABBCollider();
    ASSERT_NE(aabb, nullptr);
    EXPECT_FLOAT_EQ(aabb->halfExtents.x, 50.f);   // Half of 100
    EXPECT_FLOAT_EQ(aabb->halfExtents.y, 25.f);   // Half of 50
}

TEST(ColliderTest, GetWrongColliderTypeReturnsNull) {
    Object obj;
    obj.SetCircleCollider(20.f);
    
    EXPECT_EQ(obj.GetAABBCollider(), nullptr);
    EXPECT_NE(obj.GetCircleCollider(), nullptr);
}

