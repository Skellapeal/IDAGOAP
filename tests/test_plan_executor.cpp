//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "plan_executor.h"
#include "rida_planner.h"
#include "heuristics.h"

using namespace rida_goap;

class instant_action : public goap_action
{
public:
    instant_action(const std::string& n, const int c) : goap_action(n, c) {}
    action_status on_tick(float) override { return action_status::Succeeded; }
};

class failing_action : public goap_action
{
public:
    failing_action(const std::string& n, const int c) : goap_action(n, c) {}
    action_status on_tick(float) override { return action_status::Failed; }
};

class multi_tick_action : public goap_action
{
    int ticks_remaining;
public:
    multi_tick_action(const std::string& n, const int c, const int ticks)
        : goap_action(n, c), ticks_remaining(ticks) {}
    action_status on_tick(float) override
    {
        if (--ticks_remaining <= 0) return action_status::Succeeded;
        return action_status::Running;
    }
};

static plan_result build_plan(const std::vector<goap_action::ptr> &actions)
{
    plan_result r;
    r.status = plan_status::Success;
    for (auto& a : actions)
        r.actions.push_back(a);
    return r;
}

TEST(PlanExecutor, InitialStatusIsIdle)
{
    world_state ws;
    const plan_executor ex(&ws);
    EXPECT_EQ(ex.get_status(), execution_status::Idle);
}

TEST(PlanExecutor, SetPlanMovesToRunning)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto a = std::make_shared<instant_action>("act", 1);
    a->add_effect("done", state_value{true});
    ex.set_plan(build_plan({a}), goal);

    auto result = ex.tick(0.016f);
    EXPECT_EQ(ex.get_status(), execution_status::Running);
}

TEST(PlanExecutor, SingleActionPlanReachesSuccess)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto a = std::make_shared<instant_action>("act", 1);
    a->add_effect("done", state_value{true});
    ex.set_plan(build_plan({a}), goal);

    ex.tick(0.016f);
    ex.tick(0.016f);

    EXPECT_EQ(ex.get_status(), execution_status::Success);
}

TEST(PlanExecutor, FailingActionTransitionsToFailed)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);
    ex.set_auto_replan(false);

    auto a = std::make_shared<failing_action>("fail", 1);
    ex.set_plan(build_plan({a}), goal);

    ex.tick(0.016f);
    ex.tick(0.016f);

    EXPECT_EQ(ex.get_status(), execution_status::Failed);
}

TEST(PlanExecutor, MultiTickActionRunsUntilComplete)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto a = std::make_shared<multi_tick_action>("slow", 1, 3);
    a->add_effect("done", state_value{true});
    ex.set_plan(build_plan({a}), goal);

    ex.tick(0.016f);
    EXPECT_EQ(ex.get_status(), execution_status::Running);
    ex.tick(0.016f);
    EXPECT_EQ(ex.get_status(), execution_status::Running);
    ex.tick(0.016f);
    ex.tick(0.016f);
    EXPECT_EQ(ex.get_status(), execution_status::Success);
}

TEST(PlanExecutor, InterruptSetsInterruptedStatus)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto a = std::make_shared<multi_tick_action>("slow", 1, 10);
    ex.set_plan(build_plan({a}), goal);

    ex.tick(0.016f);
    ex.interrupt();
    EXPECT_EQ(ex.get_status(), execution_status::Interrupted);
}

TEST(PlanExecutor, ResetReturnsToIdle)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto a = std::make_shared<instant_action>("act", 1);
    ex.set_plan(build_plan({a}), goal);
    ex.tick(0.016f);
    ex.reset();

    EXPECT_EQ(ex.get_status(), execution_status::Idle);
    EXPECT_EQ(ex.get_current_action_index(), 0u);
}

TEST(PlanExecutor, IsRunningAndIsCompleteFlags)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    EXPECT_FALSE(ex.is_running());
    EXPECT_FALSE(ex.is_complete());

    auto a = std::make_shared<instant_action>("act", 1);
    a->add_effect("done", state_value{true});
    ex.set_plan(build_plan({a}), goal);

    EXPECT_TRUE(ex.is_running());
    EXPECT_FALSE(ex.is_complete());

    ex.tick(0.016f);
    ex.tick(0.016f);

    EXPECT_FALSE(ex.is_running());
    EXPECT_TRUE(ex.is_complete());
}

TEST(PlanExecutor, ReplanCallbackIsInvokedOnActionFailure)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);
    ex.set_auto_replan(true);

    bool replan_called = false;
    ex.set_replan_callback([&](const world_state&, const world_state&) -> plan_result
    {
        replan_called = true;
        plan_result empty;
        empty.status = plan_status::NoSolutionExists;
        return empty;
    });

    auto a = std::make_shared<failing_action>("fail", 1);
    ex.set_plan(build_plan({a}), goal);

    ex.tick(0.016f);
    ex.tick(0.016f);

    EXPECT_TRUE(replan_called);
}

TEST(PlanExecutor, EmptyPlanIsTriviallySuccessful)
{
    world_state ws, goal;
    ws.set_bool("done", true);
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    plan_result empty_plan;
    empty_plan.status = plan_status::Success;
    ex.set_plan(empty_plan, goal);

    ex.tick(0.016f);
    EXPECT_EQ(ex.get_status(), execution_status::Success);
}