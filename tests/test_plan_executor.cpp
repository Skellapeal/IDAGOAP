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

class tracked_action : public goap_action
{
public:
    bool started = false, ended = false, interrupted = false, end_success = false;

    tracked_action(const std::string& n, const int c) : goap_action(n, c) {}
    action_status on_tick(float) override { return action_status::Succeeded; }

    bool on_start() override { started = true; return true; }
    void on_end(const bool s) override { ended = true; end_success = s; }
    void on_interrupt() override { interrupted = true; }
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

    auto a = std::make_shared<multi_tick_action>("act", 1, 5);
    ex.set_plan(build_plan({a}), goal);

    ex.tick(0.016f);
    EXPECT_EQ(ex.get_status(), execution_status::Running);
}

TEST(PlanExecutor, SetPlanMovesToRunningBeforeTick)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto a = std::make_shared<instant_action>("act", 1);
    ex.set_plan(build_plan({a}), goal);
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
    EXPECT_EQ(ex.get_status(), execution_status::Failed);
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

TEST(PlanExecutor, SetPlanInterruptsRunningPlan)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto slow = std::make_shared<multi_tick_action>("slow", 1, 10);
    ex.set_plan(build_plan({slow}), goal);
    ex.tick(0.016f);

    auto fast = std::make_shared<instant_action>("fast", 1);
    fast->add_effect("done", state_value{true});
    ex.set_plan(build_plan({fast}), goal);

    EXPECT_EQ(ex.get_status(), execution_status::Running);
    EXPECT_EQ(ex.get_current_action_index(), 0u);
    ex.tick(0.016f); ex.tick(0.016f);
    EXPECT_EQ(ex.get_status(), execution_status::Success);
}

TEST(PlanExecutor, PreconditionFailureAtRuntimeTriggersReplan)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    ws.set_bool("ready", false);
    plan_executor ex(&ws); ex.set_auto_replan(false);

    auto a = std::make_shared<instant_action>("guarded", 1);
    a->add_precondition("ready", state_value{true});
    ex.set_plan(build_plan({a}), goal);
    ex.tick(0.016f);

    EXPECT_EQ(ex.get_status(), execution_status::Failed);
}

TEST(PlanExecutor, NullWorldModelCausesImmediateFail)
{
    plan_executor ex(nullptr);
    world_state goal;
    goal.set_bool("done", true);

    const auto a = std::make_shared<instant_action>("act", 1);
    plan_result r; r.status = plan_status::Success; r.actions.push_back(a);
    ex.set_plan(r, goal);
    const auto result = ex.tick(0.016f);

    EXPECT_EQ(ex.get_status(), execution_status::Failed);
    EXPECT_FALSE(result.failure_reason.empty());
}

TEST(PlanExecutor, LifecycleCallbacksInvokedByExecutor)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto t = std::make_shared<tracked_action>("t", 1);
    t->add_effect("done", state_value{true});
    ex.set_plan(build_plan({t}), goal);
    ex.tick(0.016f);

    EXPECT_TRUE(t->started);
    EXPECT_TRUE(t->ended);
    EXPECT_TRUE(t->end_success);
    EXPECT_FALSE(t->interrupted);
}

TEST(PlanExecutor, InterruptCallsOnInterruptNotOnEnd)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto t = std::make_shared<tracked_action>("t", 1);
    auto slow = std::make_shared<multi_tick_action>("slow", 1, 10);

    class lifecycle_observer : public goap_action
    {
    public:
        int start_count     = 0;
        int end_count       = 0;
        int interrupt_count = 0;

        lifecycle_observer() : goap_action("obs", 1) {}
        action_status on_tick(float) override { return action_status::Running; }
        bool on_start() override { ++start_count; return true; }
        void on_end(bool) override { ++end_count; }
        void on_interrupt() override { ++interrupt_count; }
    };

    auto obs = std::make_shared<lifecycle_observer>();
    ex.set_plan(build_plan({obs}), goal);
    ex.tick(0.016f);

    EXPECT_EQ(obs->start_count, 1);
    EXPECT_EQ(obs->end_count, 0);

    ex.interrupt();

    EXPECT_EQ(obs->interrupt_count, 1);
    EXPECT_EQ(obs->end_count, 0);
    EXPECT_EQ(ex.get_status(), execution_status::Interrupted);
}

TEST(PlanExecutor, SuccessReportedSameTickLastActionCompletes)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto a = std::make_shared<instant_action>("act", 1);
    a->add_effect("done", state_value{true});
    ex.set_plan(build_plan({a}), goal);

    const auto result = ex.tick(0.016f);

    EXPECT_EQ(result.status, execution_status::Success);
    EXPECT_EQ(ex.get_status(), execution_status::Success);
}