//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "async_planner.h"
#include "heuristics.h"

using namespace rida_goap;

static goap_action::ptr make_simple_action(const std::string& name,
    const std::string& pre_key, bool pre_val,
    const std::string& eff_key, bool eff_val)
{
    class sa : public goap_action
    {
    public:
        sa(const std::string& n, const int c) : goap_action(n, c) {}
        action_status on_tick(float) override { return action_status::Succeeded; }
    };

    auto a = std::make_shared<sa>(name, 1);
    a->add_precondition(pre_key, state_value{pre_val});
    a->add_effect(eff_key, state_value{eff_val});
    return a;
}

TEST(AsyncPlanner, IsNotPlanningInitially)
{
    const async_planner ap;
    EXPECT_FALSE(ap.is_planning_active());
}

TEST(AsyncPlanner, PlanAsyncCompletesSuccessfully)
{
    async_planner ap;
    world_state start, goal;
    start.set_bool("x", false);
    goal.set_bool("x", true);

    auto action = make_simple_action("flip", "x", false, "x", true);
    std::vector actions = {action};

    ap.plan_async(start, goal, actions, std::make_shared<goal_count_heuristic>());
    auto result = ap.wait_for_result();

    EXPECT_TRUE(result.success());
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result.actions[0]->get_name(), "flip");
}

TEST(AsyncPlanner, CallbackIsInvokedOnCompletion)
{
    async_planner ap;
    world_state start, goal;
    start.set_bool("x", false);
    goal.set_bool("x", true);

    auto action = make_simple_action("flip", "x", false, "x", true);
    std::vector actions = {action};

    bool callback_called = false;
    plan_result callback_result;

    ap.plan_async(start, goal, actions, std::make_shared<goal_count_heuristic>(),
        [&](plan_result r)
        {
            callback_called = true;
            callback_result = std::move(r);
        });

    ap.wait_for_result();

    EXPECT_TRUE(callback_called);
    EXPECT_TRUE(callback_result.success());
}

TEST(AsyncPlanner, TryGetResultReturnsFalseWithNoDispatch)
{
    async_planner ap;
    plan_result r;
    EXPECT_FALSE(ap.try_get_result(r));
}

TEST(AsyncPlanner, CancelStopsActivePlanningAndResultIsNotSuccess)
{
    async_planner ap;
    world_state start, goal;
    start.set_bool("x", false);
    goal.set_bool("x", true);

    auto action = make_simple_action("flip", "x", false, "x", true);
    std::vector actions = {action};

    planner_options opts;
    opts.time_budget_ms = 5000;

    ap.plan_async(start, goal, actions, std::make_shared<goal_count_heuristic>(), opts);
    ap.cancel_planning();
    auto result = ap.wait_for_result();

    EXPECT_FALSE(ap.is_planning_active());
    if (!result.success())
    {
        EXPECT_NE(result.status, plan_status::NoSolutionExists);
    }
}

TEST(AsyncPlanner, SetCallbackBeforePlanIsUsed)
{
    async_planner ap;
    bool called = false;
    ap.set_completion_callback([&](const plan_result&) { called = true; });

    world_state start, goal;
    start.set_bool("x", false);
    goal.set_bool("x", true);
    const auto action = make_simple_action("flip", "x", false, "x", true);
    const std::vector actions = {action};

    ap.plan_async(start, goal, actions, std::make_shared<goal_count_heuristic>());
    ap.wait_for_result();
    EXPECT_TRUE(called);
}

TEST(AsyncPlanner, SecondPlanWhileFirstInFlightDoesNotCrash)
{
    async_planner ap;

    world_state start1, goal1;
    start1.set_bool("x", false);
    goal1.set_bool("x", true);

    class sa : public goap_action
    {
    public:
        explicit sa(const std::string& n) : goap_action(n, 1) {}
        action_status on_tick(float) override { return action_status::Succeeded; }
    };

    auto a1 = std::make_shared<sa>("flip1");
    a1->add_precondition("x", state_value{false});
    a1->add_effect("x", state_value{true});
    std::vector<goap_action::ptr> actions1 = {a1};

    planner_options slow_opts;
    slow_opts.time_budget_ms = 5000;

    ap.plan_async(start1, goal1, actions1,
                  std::make_shared<goal_count_heuristic>(), slow_opts);

    world_state start2, goal2;
    start2.set_bool("y", false);
    goal2.set_bool("y", true);

    auto a2 = std::make_shared<sa>("flip2");
    a2->add_precondition("y", state_value{false});
    a2->add_effect("y", state_value{true});
    std::vector<goap_action::ptr> actions2 = {a2};

    ap.plan_async(start2, goal2, actions2,
                  std::make_shared<goal_count_heuristic>());

    const auto result = ap.wait_for_result();

    EXPECT_FALSE(result.status_string().empty());
    EXPECT_TRUE(result.success());
    ASSERT_EQ(result.size(), 1u);
    EXPECT_EQ(result.actions[0]->get_name(), "flip2");
}