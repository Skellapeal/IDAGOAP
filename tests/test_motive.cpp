//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "motive.h"

using namespace rida_goap;

TEST(Motive, GetPriorityReturnsConstructedValue)
{
    world_state goal;
    goal.set_bool("alive", true);
    const motive m("", goal, 5);
    EXPECT_EQ(m.get_priority(), 5);
}

TEST(Motive, GetGoalStateMatchesConstructed)
{
    world_state goal;
    goal.set_bool("enemy_dead", true);
    const motive m("", goal, 1);
    EXPECT_TRUE(m.get_goal_state().has_state("enemy_dead"));
}

TEST(Motive, IsSatisfiedWhenWorldMatchesGoal)
{
    world_state goal;
    goal.set_bool("enemy_dead", true);
    const motive m("", goal, 1);
    world_state world;
    world.set_bool("enemy_dead", true);
    EXPECT_TRUE(m.is_satisfied(world));
}

TEST(Motive, IsNotSatisfiedWhenWorldDiffers)
{
    world_state goal;
    goal.set_bool("enemy_dead", true);
    const motive m("", goal, 1);
    world_state world;
    world.set_bool("enemy_dead", false);
    EXPECT_FALSE(m.is_satisfied(world));
}

TEST(Motive, IsNotSatisfiedWhenWorldMissingKey)
{
    world_state goal;
    goal.set_bool("enemy_dead", true);
    const motive m("", goal, 1);
    const world_state world;
    EXPECT_FALSE(m.is_satisfied(world));
}

TEST(Motive, IsSatisfiedWithEmptyGoalIsAlwaysTrue)
{
    const world_state goal;
    const motive m("", goal, 0);
    world_state world;
    world.set_bool("alive", true);
    EXPECT_TRUE(m.is_satisfied(world));
}
