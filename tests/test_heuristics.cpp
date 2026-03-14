//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "heuristics.h"

using namespace rida_goap;

TEST(ZeroHeuristic, AlwaysReturnsZero)
{
    const zero_heuristic h;
    world_state ws, goal;
    ws.set_bool("alive", true);
    goal.set_bool("alive", false);
    EXPECT_EQ(h.estimate(ws, goal), 0);
}

TEST(GoalCountHeuristic, ReturnsZeroWhenGoalSatisfied)
{
    const goal_count_heuristic h;
    world_state ws, goal;
    ws.set_bool("alive", true);
    goal.set_bool("alive", true);
    EXPECT_EQ(h.estimate(ws, goal), 0);
}

TEST(GoalCountHeuristic, CountsUnsatisfiedKeys)
{
    const goal_count_heuristic h;
    const world_state ws;
    world_state goal;
    goal.set_bool("alive", true);
    goal.set_int("ammo", 5);
    EXPECT_EQ(h.estimate(ws, goal), 2);
}

TEST(GoalCountHeuristic, CountsPartiallyUnsatisfied)
{
    const goal_count_heuristic h;
    world_state ws, goal;
    ws.set_bool("alive", true);
    goal.set_bool("alive", true);
    goal.set_int("ammo", 5);
    EXPECT_EQ(h.estimate(ws, goal), 1);
}

TEST(GoalCountHeuristic, CountsValueMismatch)
{
    const goal_count_heuristic h;
    world_state ws, goal;
    ws.set_bool("alive", false);
    goal.set_bool("alive", true);
    EXPECT_EQ(h.estimate(ws, goal), 1);
}

TEST(GoalCountHeuristic, EmptyGoalReturnsZero)
{
    const goal_count_heuristic h;
    world_state ws;
    const world_state goal;
    ws.set_bool("alive", true);
    EXPECT_EQ(h.estimate(ws, goal), 0);
}

TEST(EuclideanHeuristic, Returns0WhenAtGoal)
{
    const euclidean_heuristic h;
    world_state ws, goal;
    ws.set_position("position", 0.0f, 0.0f, 0.0f);
    goal.set_position("position", 0.0f, 0.0f, 0.0f);
    EXPECT_EQ(h.estimate(ws, goal), 0);
}

TEST(EuclideanHeuristic, ComputesDistanceCorrectly)
{
    const euclidean_heuristic h;
    world_state ws, goal;
    ws.set_position("position", 0.0f, 0.0f, 0.0f);
    goal.set_position("position", 3.0f, 4.0f, 0.0f);
    EXPECT_EQ(h.estimate(ws, goal), 5);
}

TEST(EuclideanHeuristic, Returns0WhenPositionKeyMissing)
{
    const euclidean_heuristic h;
    const world_state ws;
    const world_state goal;
    EXPECT_EQ(h.estimate(ws, goal), 0);
}

TEST(EuclideanHeuristic, CustomKey)
{
    const euclidean_heuristic h("loc");
    world_state ws, goal;
    ws.set_position("loc", 0.0f, 0.0f, 0.0f);
    goal.set_position("loc", 0.0f, 3.0f, 4.0f);
    EXPECT_EQ(h.estimate(ws, goal), 5);
}

TEST(ManhattanHeuristic, Returns0WhenAtGoal)
{
    const manhattan_heuristic h;
    world_state ws, goal;
    ws.set_position("position", 1.0f, 1.0f, 0.0f);
    goal.set_position("position", 1.0f, 1.0f, 0.0f);
    EXPECT_EQ(h.estimate(ws, goal), 0);
}

TEST(ManhattanHeuristic, ComputesCorrectly)
{
    const manhattan_heuristic h;
    world_state ws, goal;
    ws.set_position("position", 0.0f, 0.0f, 0.0f);
    goal.set_position("position", 3.0f, 4.0f, 0.0f);
    EXPECT_EQ(h.estimate(ws, goal), 7);
}

TEST(ManhattanHeuristic, Returns0WhenKeyMissing)
{
    const manhattan_heuristic h;
    const world_state ws;
    const world_state goal;
    EXPECT_EQ(h.estimate(ws, goal), 0);
}

TEST(CompositeHeuristic, EmptyCompositeReturnsZero)
{
    const composite_heuristic h;
    const world_state ws;
    const world_state goal;
    EXPECT_EQ(h.estimate(ws, goal), 0);
}

TEST(CompositeHeuristic, SingleHeuristicMatchesOriginal)
{
    const auto inner = std::make_shared<goal_count_heuristic>();
    composite_heuristic h;
    h.add_heuristic(inner, 1.0f);
    const world_state ws;
    world_state goal;
    goal.set_bool("alive", true);
    goal.set_bool("armed", true);
    EXPECT_EQ(h.estimate(ws, goal), 2);
}

TEST(CompositeHeuristic, WeightScalesResult)
{
    const auto inner = std::make_shared<goal_count_heuristic>();
    composite_heuristic h;
    h.add_heuristic(inner, 2.0f);
    const world_state ws;
    world_state goal;
    goal.set_bool("alive", true); // 1 unsatisfied * weight 2 = 2
    EXPECT_EQ(h.estimate(ws, goal), 2);
}