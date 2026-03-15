//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "goal_selector.h"

using namespace rida_goap;

static std::shared_ptr<motive> make_motive(const std::string& key, const bool val, int priority)
{
    world_state goal;
    goal.set_bool(key, val);
    return std::make_shared<motive>(goal, priority);
}

TEST(GoalSelector, SelectsHighestPriorityMotive)
{
    goal_selector gs;
    gs.add_motive(make_motive("patrol", true, 1));
    gs.add_motive(make_motive("attack", true, 5));
    gs.add_motive(make_motive("flee", true, 3));

    const world_state ws;
    const auto selected = gs.select_goal(ws);
    ASSERT_NE(selected, nullptr);
    EXPECT_TRUE(selected->get_goal_state().has_state("attack"));
}

TEST(GoalSelector, ReturnsNullptrWhenNoMotives)
{
    const goal_selector gs;
    const world_state ws;
    EXPECT_EQ(gs.select_goal(ws), nullptr);
}

TEST(GoalSelector, AddAndRemoveMotive)
{
    goal_selector gs;
    const auto m = make_motive("attack", true, 5);
    gs.add_motive(m);
    gs.remove_motive(m);
    const world_state ws;
    EXPECT_EQ(gs.select_goal(ws), nullptr);
}

TEST(GoalSelector, ClearRemovesAllMotives)
{
    goal_selector gs;
    gs.add_motive(make_motive("patrol", true, 1));
    gs.add_motive(make_motive("attack", true, 5));
    gs.clear_motives();
    EXPECT_TRUE(gs.get_motives().empty());
}

TEST(GoalSelector, CustomUtilityEvaluatorIsUsed)
{
    goal_selector gs;
    gs.add_motive(make_motive("patrol", true, 10));
    gs.add_motive(make_motive("flee", true, 1));

    gs.set_utility_evaluator([](const motive& m, const world_state&)
    {
        return m.get_goal_state().has_state("flee") ? 100.0f : 0.0f;
    });

    const world_state ws;
    const auto selected = gs.select_goal(ws);
    ASSERT_NE(selected, nullptr);
    EXPECT_TRUE(selected->get_goal_state().has_state("flee"));
}

TEST(GoalSelector, EvaluateAllMotivesReturnsAllWithScores)
{
    goal_selector gs;
    gs.add_motive(make_motive("patrol", true, 2));
    gs.add_motive(make_motive("attack", true, 7));

    const world_state ws;
    const auto all = gs.evaluate_all_motives(ws);
    ASSERT_EQ(all.size(), 2u);
    bool found_patrol = false, found_attack = false;

    for (const auto &m: all | std::views::keys)
    {
        if (m->get_goal_state().has_state("patrol")) found_patrol = true;
        if (m->get_goal_state().has_state("attack")) found_attack = true;
    }

    EXPECT_TRUE(found_patrol);
    EXPECT_TRUE(found_attack);
}

TEST(GoalSelector, SelectGoalStateReturnsNulloptWhenEmpty)
{
    const goal_selector gs;
    const world_state ws;
    EXPECT_FALSE(gs.select_goal_state(ws).has_value());
}

TEST(GoalSelector, SelectGoalStateReturnsGoalWhenPresent)
{
    goal_selector gs;
    gs.add_motive(make_motive("alive", true, 1));
    const world_state ws;
    const auto state = gs.select_goal_state(ws);
    ASSERT_TRUE(state.has_value());
    EXPECT_TRUE(state->has_state("alive"));
}
