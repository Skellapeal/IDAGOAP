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
    class sa : public goap_action {
    public: sa(const std::string& n, const int c) : goap_action(n, c) {}
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
    std::vector<goap_action::ptr> actions = {action};

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
    std::vector<goap_action::ptr> actions = {action};

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

TEST(AsyncPlanner, TryGetResultReturnsFalseBeforeDone)
{
    async_planner ap;
    // Don't dispatch anything; no future exists
    plan_result r;
    EXPECT_FALSE(ap.try_get_result(r));
}

TEST(AsyncPlanner, CancelStopsActivePlanning)
{
    async_planner ap;
    world_state start, goal;
    start.set_bool("x", false);
    goal.set_bool("x", true);

    auto action = make_simple_action("flip", "x", false, "x", true);
    std::vector<goap_action::ptr> actions = {action};

    planner_options opts;
    opts.time_budget_ms = 5000; // long budget

    ap.plan_async(start, goal, actions, std::make_shared<goal_count_heuristic>(), opts);
    ap.cancel_planning();
    auto result = ap.wait_for_result();

    // After cancel, it should not be active
    EXPECT_FALSE(ap.is_planning_active());
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
