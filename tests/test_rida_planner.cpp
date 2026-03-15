//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "rida_planner.h"
#include "heuristics.h"

using namespace rida_goap;

static goap_action::ptr make_action(
    const std::string& name, int cost,
    std::initializer_list<std::pair<std::string, state_value>> preconditions,
    std::initializer_list<std::pair<std::string, state_value>> effects)
{
    class simple_action : public goap_action
    {
    public:
        simple_action(const std::string& n, const int c) : goap_action(n, c) {}
    };
    auto a = std::make_shared<simple_action>(name, cost);
    for (auto& [k, v] : preconditions) a->add_precondition(k, v);
    for (auto& [k, v] : effects)       a->add_effect(k, v);
    return a;
}

TEST(RidaPlanner, TriviallySatisfiedGoalReturnEmptyPlan)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("alive", true);
    goal.set_bool("alive", true);
    std::vector<goap_action::ptr> actions;
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});
    EXPECT_TRUE(result.success());
    EXPECT_TRUE(result.is_trivially_satisfied());
}

TEST(RidaPlanner, SingleActionPlan)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("has_weapon", false);
    goal.set_bool("has_weapon", true);

    const auto pick_up = make_action("pick_up_weapon", 1,
        {{"has_weapon", state_value{false}}},
        {{"has_weapon", state_value{true}}});

    std::vector actions = {pick_up};
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});

    ASSERT_TRUE(result.success());
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result.actions[0]->get_name(), "pick_up_weapon");
}

TEST(RidaPlanner, MultiStepPlan)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("has_weapon", false);
    start.set_bool("enemy_dead", false);
    goal.set_bool("enemy_dead", true);

    const auto pick_up  = make_action("pick_up_weapon", 1,
        {{"has_weapon", state_value{false}}},
        {{"has_weapon", state_value{true}}});

    const auto shoot    = make_action("shoot_enemy", 1,
        {{"has_weapon", state_value{true}}},
        {{"enemy_dead", state_value{true}}});

    std::vector actions = {pick_up, shoot};
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});

    ASSERT_TRUE(result.success());
    EXPECT_EQ(result.size(), 2u);
}

TEST(RidaPlanner, NoSolutionWhenActionsCannotSatisfyGoal)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("alive", true);
    goal.set_bool("enemy_dead", true);

    const auto irrelevant = make_action("patrol", 1, {}, {{"patrolling", state_value{true}}});
    std::vector actions = {irrelevant};

    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});
    EXPECT_FALSE(result.success());
    EXPECT_EQ(result.status, plan_status::NoSolutionExists);
}

TEST(RidaPlanner, DepthLimitPreventsPlan)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("has_weapon", false);
    start.set_bool("enemy_dead", false);
    goal.set_bool("enemy_dead", true);

    auto pick_up = make_action("pick_up_weapon", 1,
        {{"has_weapon", state_value{false}}},
        {{"has_weapon", state_value{true}}});
    auto shoot   = make_action("shoot_enemy", 1,
        {{"has_weapon", state_value{true}}},
        {{"enemy_dead", state_value{true}}});

    std::vector actions = {pick_up, shoot};
    planner_options opts;
    opts.max_depth = 1;

    auto result = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);
    EXPECT_FALSE(result.success());
    EXPECT_EQ(result.status, plan_status::DepthLimitReached);
}

TEST(RidaPlanner, DepthLimitOf2AllowsTwoStepPlan)
{
    rida_planner planner;
    world_state start, goal;

    start.set_bool("has_weapon", false); start.set_bool("enemy_dead", false);
    goal.set_bool("enemy_dead", true);

    auto pick_up = make_action("pick_up_weapon", 1,
        {{"has_weapon", state_value{false}}}, {{"has_weapon", state_value{true}}});
    auto shoot   = make_action("shoot_enemy",   1,
        {{"has_weapon", state_value{true}}},  {{"enemy_dead", state_value{true}}});

    std::vector actions = {pick_up, shoot};
    planner_options opts; opts.max_depth = 2;
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);

    EXPECT_TRUE(result.success());
    EXPECT_EQ(result.size(), 2u);
}

TEST(RidaPlanner, NodeLimitTriggersEarlyExit)
{
    rida_planner planner;
    world_state start, goal;

    start.set_bool("has_weapon", false); start.set_bool("enemy_dead", false);
    goal.set_bool("enemy_dead", true);

    auto pick_up = make_action("pick_up_weapon", 1,
        {{"has_weapon", state_value{false}}}, {{"has_weapon", state_value{true}}});
    auto shoot   = make_action("shoot_enemy",   1,
        {{"has_weapon", state_value{true}}},  {{"enemy_dead", state_value{true}}});

    std::vector actions = {pick_up, shoot};
    planner_options opts; opts.max_nodes = 1;
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);

    EXPECT_FALSE(result.success());
    EXPECT_EQ(result.status, plan_status::NodeLimitReached);
}

TEST(RidaPlanner, PlanSucceedsWithTranspositionTableDisabled)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("x", false);
    goal.set_bool("x", true);

    const auto a = make_action("flip", 1, {{"x", state_value{false}}}, {{"x", state_value{true}}});
    std::vector actions = {a};

    planner_options opts;
    opts.use_transposition_table = false;

    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);
    EXPECT_TRUE(result.success());
}

TEST(RidaPlanner, CancelTokenAbortsPlanning)
{
    rida_planner planner;
    const std::atomic cancel{true};
    world_state start, goal;

    start.set_bool("a", false); start.set_bool("b", false); start.set_bool("c", false);
    goal.set_bool("c", true);

    auto s1 = make_action("s1",1,{{"a",state_value{false}}},{{"a",state_value{true}}});
    auto s2 = make_action("s2",1,{{"a",state_value{true}}}, {{"b",state_value{true}}});
    auto s3 = make_action("s3",1,{{"b",state_value{true}}}, {{"c",state_value{true}}});

    std::vector actions = {s1,s2,s3};
    planner_options opts; opts.cancel_token = &cancel;
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);

    EXPECT_FALSE(result.success());
    EXPECT_NE(result.status, plan_status::NoSolutionExists);
}

TEST(RidaPlanner, ChoosesCheaperPlanWhenTwoPathsExist)
{
    rida_planner planner;
    world_state start, goal;

    start.set_bool("enemy_dead", false); goal.set_bool("enemy_dead", true);

    const auto expensive = make_action("bomb",  10, {}, {{"enemy_dead", state_value{true}}});
    const auto cheap     = make_action("shoot",  1, {}, {{"enemy_dead", state_value{true}}});

    std::vector actions = {expensive, cheap};
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});

    ASSERT_TRUE(result.success());
    EXPECT_EQ(result.actions[0]->get_name(), "shoot");
    EXPECT_EQ(result.final_cost, 1);
}

TEST(RidaPlanner, CyclicActionsWithTTDoNotLoopInfinitely)
{
    rida_planner planner;
    world_state start, goal;

    start.set_bool("flag", false);
    goal.set_bool("unreachable", true);

    auto set_true  = make_action("set_true",  1,
        {{"flag", state_value{false}}}, {{"flag", state_value{true}}});
    auto set_false = make_action("set_false", 1,
        {{"flag", state_value{true}}},  {{"flag", state_value{false}}});

    std::vector actions = {set_true, set_false};
    planner_options opts;

    opts.use_transposition_table = true;
    opts.max_nodes = 200;
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);

    EXPECT_FALSE(result.success());
    EXPECT_TRUE(result.status == plan_status::NoSolutionExists || result.status == plan_status::NodeLimitReached
    );
}