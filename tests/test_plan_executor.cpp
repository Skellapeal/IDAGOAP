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

class many_tick_action : public goap_action
{
    int ticks_remaining;
public:
    many_tick_action(const std::string& n, const int c, const int ticks)
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

    auto a = std::make_shared<many_tick_action>("slow", 1, 3);
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

    auto a = std::make_shared<many_tick_action>("slow", 1, 10);
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

    auto slow = std::make_shared<many_tick_action>("slow", 1, 10);
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
    auto slow = std::make_shared<many_tick_action>("slow", 1, 10);

    class lifecycle_observer : public goap_action
    {
    public:
        int start_count = 0;
        int end_count = 0;
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

    ex.tick(0.016f);
    const auto result = ex.tick(0.016f);

    EXPECT_EQ(result.status, execution_status::Success);
    EXPECT_EQ(ex.get_status(), execution_status::Success);
}

TEST(PlanExecutor, EffectsAppliedToWorldOnActionSuccess)
{
    world_state ws, goal;
    ws.set_bool("has_weapon", false);
    goal.set_bool("enemy_dead", true);
    plan_executor ex(&ws);

    auto a = std::make_shared<instant_action>("pickup", 1);
    a->add_precondition("has_weapon", state_value{false});
    a->add_effect("has_weapon", state_value{true});
    ex.set_plan(build_plan({a}), goal);

    ex.tick(0.016f);
    ex.tick(0.016f);

    ASSERT_TRUE(ws.get_bool("has_weapon").has_value());
    EXPECT_TRUE(*ws.get_bool("has_weapon"));
}

TEST(PlanExecutor, FailureReasonClearedAfterSuccessfulReplan)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);
    ex.set_auto_replan(true);

    ex.set_replan_callback([&](const world_state&, const world_state&) -> plan_result
    {
        const auto replacement = std::make_shared<instant_action>("replacement", 1);
        replacement->add_effect("done", state_value{true});
        plan_result r;
        r.status = plan_status::Success;
        r.actions.push_back(replacement);
        return r;
    });

    auto failing = std::make_shared<failing_action>("fail", 1);
    ex.set_plan(build_plan({failing}), goal);

    const auto result = ex.tick(0.016f);

    EXPECT_TRUE(result.failure_reason.empty())
        << "Stale failure_reason after replan: " << result.failure_reason;
    EXPECT_EQ(result.status, execution_status::Running);
}

TEST(PlanExecutor, OnEndNotCalledWhenOnStartFails)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    bool end_called = false;

    class start_failing_action : public goap_action {
    public:
        bool* end_flag;
        start_failing_action(bool* flag) : goap_action("fail_start", 1), end_flag(flag) {}
        bool on_start() override { return false; }
        action_status on_tick(float) override { return action_status::Succeeded; }
        void on_end(bool) override { *end_flag = true; }
    };

    auto a = std::make_shared<start_failing_action>(&end_called);
    ex.set_plan(build_plan({a}), goal);
    ex.tick(0.016f);

    EXPECT_FALSE(end_called) << "on_end must not be called if on_start returned false";
    EXPECT_EQ(ex.get_status(), execution_status::Failed);
}

TEST(PlanExecutor, ResetAfterSuccessAllowsNewPlan)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto a = std::make_shared<instant_action>("act", 1);
    a->add_effect("done", state_value{true});

    ex.set_plan(build_plan({a}), goal);
    while (ex.is_running()) ex.tick(0.016f);

    ASSERT_EQ(ex.get_status(), execution_status::Success);

    ex.reset();
    EXPECT_EQ(ex.get_status(), execution_status::Idle);

    auto b = std::make_shared<instant_action>("act2", 1);
    b->add_effect("done", state_value{true});

    ex.set_plan(build_plan({b}), goal);
    EXPECT_EQ(ex.get_status(), execution_status::Running);

    while (ex.is_running()) ex.tick(0.016f);
    EXPECT_EQ(ex.get_status(), execution_status::Success);
}

TEST(PlanExecutor, FailureReasonPopulatedOnNullWorldModel)
{
    plan_executor ex(nullptr);
    world_state goal;
    goal.set_bool("done", true);

    const auto a = std::make_shared<instant_action>("act", 1);
    plan_result r;
    r.status = plan_status::Success;
    r.actions.push_back(a);

    ex.set_plan(r, goal);
    const auto result = ex.tick(0.016f);

    EXPECT_FALSE(result.failure_reason.empty());
    EXPECT_EQ(result.status, execution_status::Failed);
}

TEST(PlanExecutor, FailureReasonContainsActionNameOnActionFail)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);
    ex.set_auto_replan(false);

    auto a = std::make_shared<failing_action>("named_fail_action", 1);
    ex.set_plan(build_plan({a}), goal);

    ex.tick(0.016f);
    const auto result = ex.tick(0.016f);

    EXPECT_EQ(result.status, execution_status::Failed);
    EXPECT_NE(result.failure_reason.find("named_fail_action"), std::string::npos)
        << "Failure reason should contain the action name. Got: " << result.failure_reason;
}

TEST(PlanExecutor, FailureReasonContainsActionNameOnPreconditionFail)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    ws.set_bool("ready", false);
    plan_executor ex(&ws);
    ex.set_auto_replan(false);

    auto a = std::make_shared<instant_action>("guarded_action", 1);
    a->add_precondition("ready", state_value{true});
    ex.set_plan(build_plan({a}), goal);

    const auto result = ex.tick(0.016f);

    EXPECT_NE(result.failure_reason.find("guarded_action"), std::string::npos)
        << "Failure reason should contain the action name";
}

TEST(PlanExecutor, SuccessTickDoesNotAdvanceActionIndexBeyondPlanSize)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto a = std::make_shared<instant_action>("act", 1);
    a->add_effect("done", state_value{true});
    ex.set_plan(build_plan({a}), goal);

    ex.tick(0.016f);
    ex.tick(0.016f);
    ex.tick(0.016f);

    EXPECT_EQ(ex.get_status(), execution_status::Success);
    EXPECT_LE(ex.get_current_action_index(), 1u);
}

TEST(PlanExecutor, ReplanCallbackReceivesCurrentWorldAndGoal)
{
    world_state ws, goal;
    ws.set_bool("armed", false);
    goal.set_bool("done", true);
    plan_executor ex(&ws);
    ex.set_auto_replan(true);

    bool world_correct = false;
    bool goal_correct  = false;

    ex.set_replan_callback(
        [&](const world_state& w, const world_state& g) -> plan_result
        {
            world_correct = w.has_state("armed");
            goal_correct  = g.has_state("done");

            plan_result fail;
            fail.status = plan_status::NoSolutionExists;
            return fail;
        });

    auto a = std::make_shared<failing_action>("fail", 1);
    ex.set_plan(build_plan({a}), goal);
    ex.tick(0.016f);
    ex.tick(0.016f);

    EXPECT_TRUE(world_correct) << "Replan callback did not receive current world model";
    EXPECT_TRUE(goal_correct)  << "Replan callback did not receive current goal";
}

TEST(PlanExecutor, SuccessfulReplanResumesExecutionFromNewPlanStart)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);
    ex.set_auto_replan(true);

    int replan_count = 0;

    ex.set_replan_callback(
        [&](const world_state&, const world_state& g) -> plan_result
        {
            ++replan_count;
            const auto replacement = std::make_shared<instant_action>("recovery", 1);
            replacement->add_effect("done", state_value{true});
            plan_result r;
            r.status = plan_status::Success;
            r.actions.push_back(replacement);
            return r;
        });

    auto fail = std::make_shared<failing_action>("fail", 1);
    ex.set_plan(build_plan({fail}), goal);

    ex.tick(0.016f);
    ex.tick(0.016f);
    EXPECT_EQ(replan_count, 1);
    EXPECT_EQ(ex.get_status(), execution_status::Running);
    EXPECT_EQ(ex.get_current_action_index(), 0u)
        << "After replan, execution must restart from index 0";
}

TEST(PlanExecutor, MultiActionPlanAdvancesIndexOnEachSuccess)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto a1 = std::make_shared<instant_action>("step1", 1);
    auto a2 = std::make_shared<instant_action>("step2", 1);
    auto a3 = std::make_shared<instant_action>("step3", 1);
    a3->add_effect("done", state_value{true});

    ex.set_plan(build_plan({a1, a2, a3}), goal);

    ex.tick(0.016);
    ex.tick(0.016f);
    EXPECT_EQ(ex.get_current_action_index(), 1u);

    ex.tick(0.016);
    ex.tick(0.016f);
    EXPECT_EQ(ex.get_current_action_index(), 2u);

    ex.tick(0.016);
    ex.tick(0.016f);
    EXPECT_EQ(ex.get_status(), execution_status::Success);
}

TEST(PlanExecutor, CurrentActionReflectsActiveActionDuringExecution)
{
    world_state ws, goal;
    goal.set_bool("done", true);
    plan_executor ex(&ws);

    auto slow = std::make_shared<many_tick_action>("slow_step", 1, 3);
    ex.set_plan(build_plan({slow}), goal);

    ex.tick(0.016f); // starts slow_step
    ASSERT_NE(ex.get_current_action(), nullptr);
    EXPECT_EQ(ex.get_current_action()->get_name(), "slow_step");
}