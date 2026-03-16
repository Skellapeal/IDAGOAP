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
        action_status on_tick(float) override { return action_status::Succeeded; }
    };

    auto a = std::make_shared<simple_action>(name, cost);
    for (auto& [k, v] : preconditions) a->add_precondition(k, v);
    for (auto& [k, v] : effects) a->add_effect(k, v);
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
    std::stop_source cancel;
    world_state start, goal;

    start.set_bool("a", false);
    start.set_bool("b", false);
    start.set_bool("c", false);
    goal.set_bool("c", true);

    auto s1 = make_action("s1", 1, {{"a", state_value{false}}}, {{"a", state_value{true}}});
    auto s2 = make_action("s2", 1, {{"a", state_value{true}}},  {{"b", state_value{true}}});
    auto s3 = make_action("s3", 1, {{"b", state_value{true}}},  {{"c", state_value{true}}});

    std::vector actions = {s1, s2, s3};
    planner_options opts;
    opts.cancel_token = cancel.get_token();

    cancel.request_stop();

    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);

    EXPECT_FALSE(result.success());
    EXPECT_TRUE(
        result.status == plan_status::Cancelled ||
        result.status == plan_status::TimedOut
    ) << "Expected cancellation status, got: " << result.status_string();
}

TEST(RidaPlanner, ChoosesCheaperPlanWhenTwoPathsExist)
{
    rida_planner planner;
    world_state start, goal;

    start.set_bool("enemy_dead", false); goal.set_bool("enemy_dead", true);

    const auto expensive = make_action("bomb",  10, {}, {{"enemy_dead", state_value{true}}});
    const auto cheap = make_action("shoot",  1, {}, {{"enemy_dead", state_value{true}}});

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

TEST(RidaPlanner, TranspositionTablePersistsAcrossBounds)
{
    rida_planner planner;
    world_state start, goal;
    start.set_int("step", 0);
    goal.set_int("step", 3);

    auto s1 = make_action("s1", 1, {{"step", state_value{0}}}, {{"step", state_value{1}}});
    auto s2 = make_action("s2", 1, {{"step", state_value{1}}}, {{"step", state_value{2}}});
    auto s3 = make_action("s3", 1, {{"step", state_value{2}}}, {{"step", state_value{3}}});

    std::vector actions = {s1, s2, s3};

    planner_options opts;
    opts.use_transposition_table = true;
    opts.max_transposition_size  = 1000;

    const auto result_with_tt    = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);

    opts.use_transposition_table = false;
    const auto result_without_tt = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);

    ASSERT_TRUE(result_with_tt.success());
    ASSERT_TRUE(result_without_tt.success());
    EXPECT_EQ(result_with_tt.size(), 3u);
    EXPECT_EQ(result_without_tt.size(), 3u);
    EXPECT_LE(result_with_tt.nodes_expanded, result_without_tt.nodes_expanded);
}

TEST(RidaPlanner, ComplexDeadEndGraphReturnsNoSolution)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("a", false);
    start.set_bool("b", false);
    goal.set_bool("c", true);

    auto ab = make_action("set_b", 1, {{"a", state_value{false}}}, {{"b", state_value{true}}});
    auto ba = make_action("set_a", 1, {{"b", state_value{true}}},  {{"a", state_value{true}}});

    std::vector actions = {ab, ba};
    planner_options opts;
    opts.max_nodes = 500;

    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);
    EXPECT_FALSE(result.success());
    EXPECT_TRUE(result.status == plan_status::NoSolutionExists ||
                result.status == plan_status::NodeLimitReached);
}

TEST(RidaPlanner, DisabledActionIsNotUsedInPlan)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("enemy_dead", false);
    goal.set_bool("enemy_dead", true);

    const auto disabled = make_action("shoot", 1, {}, {{"enemy_dead", state_value{true}}});
    const auto enabled  = make_action("bomb",  5, {}, {{"enemy_dead", state_value{true}}});

    class disabled_action : public goap_action {
    public:
        disabled_action() : goap_action("shoot", 1) {}
        action_status on_tick(float) override { return action_status::Succeeded; }
        bool can_run() const override { return false; }
    };

    const auto da = std::make_shared<disabled_action>();
    da->add_effect("enemy_dead", state_value{true});

    std::vector<goap_action::ptr> actions = {da, enabled};
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});

    ASSERT_TRUE(result.success());
    EXPECT_EQ(result.actions[0]->get_name(), "bomb");
}

TEST(RidaPlanner, NodesExpandedIsFreshEachPlanCall)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("x", false);
    goal.set_bool("x", true);

    const auto a = make_action("flip", 1,
        {{"x", state_value{false}}}, {{"x", state_value{true}}});
    std::vector actions = {a};

    const auto r1 = planner.plan(start, goal, actions, goal_count_heuristic{});
    const auto r2 = planner.plan(start, goal, actions, goal_count_heuristic{});

    EXPECT_EQ(r1.nodes_expanded, r2.nodes_expanded);

    EXPECT_GT(r1.nodes_expanded, 0u);
    EXPECT_GT(r2.nodes_expanded, 0u);
    EXPECT_LT(r2.nodes_expanded, r1.nodes_expanded * 2u)
        << "r2 node count suggests accumulation from r1 — counter not reset between calls";
}

TEST(RidaPlanner, PlanFailsWhenNumericPreconditionNotMetInStartState)
{
    rida_planner planner;
    world_state start, goal;
    start.set_int("ammo", 0);
    start.set_bool("enemy_dead", false);
    goal.set_bool("enemy_dead", true);

    const auto shoot = make_action("shoot", 1,
        {{"ammo", state_value{5}}, },
        {{"enemy_dead", state_value{true}}});

    class shoot_action : public goap_action {
    public:
        shoot_action() : goap_action("shoot", 1) {
            add_precondition("ammo", state_value{5}, predicate_op::Greater);
            add_effect("enemy_dead", state_value{true});
        }
        action_status on_tick(float) override { return action_status::Succeeded; }
    };

    const auto sa = std::make_shared<shoot_action>();
    std::vector<goap_action::ptr> actions = {sa};
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});

    EXPECT_FALSE(result.success());
}

TEST(RidaPlanner, PlannerPicksActionWhosePreconditionsAreReachable)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("has_sword", false);
    start.set_bool("has_bow", true);
    goal.set_bool("enemy_dead", true);

    class sword_attack : public goap_action {
    public:
        sword_attack() : goap_action("sword_attack", 1) {
            add_precondition("has_sword", state_value{true});
            add_effect("enemy_dead", state_value{true});
        }
        action_status on_tick(float) override { return action_status::Succeeded; }
    };

    class bow_attack : public goap_action {
    public:
        bow_attack() : goap_action("bow_attack", 1) {
            add_precondition("has_bow", state_value{true});
            add_effect("enemy_dead", state_value{true});
        }
        action_status on_tick(float) override { return action_status::Succeeded; }
    };

    std::vector<goap_action::ptr> actions = {
        std::make_shared<sword_attack>(),
        std::make_shared<bow_attack>()
    };

    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});

    ASSERT_TRUE(result.success());
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result.actions[0]->get_name(), "bow_attack");
}

TEST(RidaPlanner, GoalStateIsCleanAfterBacktrack)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("a", false);
    start.set_bool("b", false);
    goal.set_bool("c", true);

    auto dead_end = make_action("dead_end", 1,
        {{"z", state_value{true}}},
        {{"c", state_value{true}}});

    auto step1 = make_action("step1", 1,
        {{"a", state_value{false}}}, {{"a", state_value{true}}});
    auto step2 = make_action("step2", 1,
        {{"a", state_value{true}}},  {{"b", state_value{true}}});
    auto correct = make_action("correct", 1,
        {{"a", state_value{true}}, {"b", state_value{true}}},
        {{"c", state_value{true}}});

    std::vector actions = {dead_end, step1, step2, correct};
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});

    ASSERT_TRUE(result.success());
    EXPECT_EQ(result.actions.back()->get_name(), "correct");
}

TEST(RidaPlanner, ZeroCostActionsDoNotHang)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("x", false);
    goal.set_bool("x", true);

    auto a = make_action("flip", 0,
        {{"x", state_value{false}}},
        {{"x", state_value{true}}});

    std::vector actions = {a};
    planner_options opts;
    opts.max_nodes = 1000;

    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{}, opts);

    EXPECT_TRUE(result.success());
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result.final_cost, 0);
}

TEST(RidaPlanner, MultiGoalPlanMinimisesCost)
{
    rida_planner planner;
    world_state start, goal;
    start.set_bool("a", false);
    start.set_bool("b", false);
    goal.set_bool("a", true);
    goal.set_bool("b", true);

    auto cheap_a     = make_action("cheap_a",     1, {}, {{"a", state_value{true}}});
    auto expensive_a = make_action("expensive_a", 10, {}, {{"a", state_value{true}}});
    auto set_b       = make_action("set_b",       1, {}, {{"b", state_value{true}}});

    std::vector actions = {expensive_a, cheap_a, set_b};
    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});

    ASSERT_TRUE(result.success());
    EXPECT_EQ(result.final_cost, 2);

    bool found_cheap    = false;
    bool found_expensive = false;
    for (const auto& a : result.actions)
    {
        if (a->get_name() == "cheap_a")     found_cheap = true;
        if (a->get_name() == "expensive_a") found_expensive = true;
    }

    EXPECT_TRUE(found_cheap)     << "cheap_a should be in the plan";
    EXPECT_FALSE(found_expensive) << "expensive_a should NOT be in the plan";
}

TEST(RidaPlanner, SortingPreservesOptimalityAcrossRepeatedCalls)
{
    rida_planner planner;
    world_state start, goal;

    start.set_bool("has_weapon", false);
    start.set_bool("enemy_dead", false);
    goal.set_bool("enemy_dead", true);

    auto expensive = make_action("bomb",  10,
        {},
        {{"enemy_dead", state_value{true}}});

    auto mid       = make_action("grenade", 5,
        {},
        {{"enemy_dead", state_value{true}}});

    auto cheap     = make_action("shoot", 1,
        {},
        {{"enemy_dead", state_value{true}}});

    std::vector actions = {expensive, mid, cheap};

    const auto result1 = planner.plan(start, goal, actions, goal_count_heuristic{});
    const auto result2 = planner.plan(start, goal, actions, goal_count_heuristic{});

    ASSERT_TRUE(result1.success());
    ASSERT_TRUE(result2.success());

    EXPECT_EQ(result1.final_cost,       result2.final_cost);

    EXPECT_EQ(result1.nodes_expanded,   result2.nodes_expanded);

    ASSERT_EQ(result1.size(), 1u);
    EXPECT_EQ(result1.actions[0]->get_name(), "shoot");
    EXPECT_EQ(result1.final_cost, 1);
}

TEST(RidaPlanner, SortingPreservesOptimalMultiStepPlan)
{
    rida_planner planner;
    world_state start, goal;

    start.set_bool("has_weapon", false);
    start.set_bool("enemy_dead", false);
    goal.set_bool("enemy_dead", true);

    auto pick_up_cheap = make_action("pick_up_pistol", 1,
        {{"has_weapon", state_value{false}}},
        {{"has_weapon", state_value{true}}});

    auto shoot_cheap   = make_action("shoot_pistol", 1,
        {{"has_weapon", state_value{true}}},
        {{"enemy_dead", state_value{true}}});

    auto airstrike     = make_action("call_airstrike", 99,
        {},
        {{"enemy_dead", state_value{true}}});

    std::vector actions = {airstrike, pick_up_cheap, shoot_cheap};

    const auto result = planner.plan(start, goal, actions, goal_count_heuristic{});

    ASSERT_TRUE(result.success());

    EXPECT_EQ(result.final_cost, 2);
    EXPECT_EQ(result.size(),     2u);

    for (const auto& action : result.actions)
    {
        EXPECT_NE(action->get_name(), "call_airstrike")
            << "Optimal plan should not include the expensive airstrike action";
    }
}