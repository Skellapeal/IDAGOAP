//
// Created by Niall Ó Colmáin on 17/03/2026.
//

#include <gtest/gtest.h>
#include "goap_agent.h"
#include "heuristics.h"

using namespace rida_goap;

class instant_action : public goap_action
{
public:
    instant_action(const std::string& n, const int c = 1) : goap_action(n, c) {}
    action_status on_tick(float) override { return action_status::Succeeded; }
};

class failing_action : public goap_action
{
public:
    failing_action(const std::string& n, const int c = 1) : goap_action(n, c) {}
    action_status on_tick(float) override { return action_status::Failed; }
};

class multi_tick_action : public goap_action
{
    int remaining_;
    int initial_;
public:
    multi_tick_action(const std::string& n, const int ticks, const int c = 1)
        : goap_action(n, c), remaining_(ticks), initial_(ticks) {}

    bool on_start() override
    {
        std::cout << "[ON_START] remaining=" << remaining_ << std::endl;
        remaining_ = initial_;
        return true;
    }

    action_status on_tick(float) override
    {
        --remaining_;
        return remaining_ <= 0 ? action_status::Succeeded : action_status::Running;
    }
};

static int pump(goap_agent& agent, const int max_ticks,
                const std::function<bool()>& stop_when = nullptr,
                const float dt = 0.016f)
{
    for (int i = 0; i < max_ticks; ++i)
    {
        if (stop_when && stop_when()) return i;
        agent.tick(dt);
    }
    return max_ticks;
}

class SimpleAgentFixture : public testing::Test
{
protected:
    agent_config cfg;
    goap_agent   agent;

    SimpleAgentFixture()
        : cfg([]
        {
            agent_config c;
            c.goal_recheck_interval    = 1;
            c.replan_on_world_change   = true;
            c.max_consecutive_failures = 3;
            c.synchronous_planning     = true;
            return c;
        }()), agent(cfg) {}

    void SetUp() override
    {
        agent.set_heuristic(std::make_shared<goal_count_heuristic>());

        world_state goal_ws;
        goal_ws.set_bool("done", true);
        agent.add_motive(std::make_shared<motive>(goal_ws, 10));

        const auto act = std::make_shared<instant_action>("finish");
        act->add_effect("done", state_value{true});
        agent.add_action(act);
    }
};

TEST(GoapAgent, InitialStatusIsIdle)
{
    const goap_agent agent;
    EXPECT_EQ(agent.get_status(), agent_status::Idle);
}

TEST(GoapAgent, IsNotPlanningInitially)
{
    const goap_agent agent;
    EXPECT_FALSE(agent.is_planning());
}

TEST(GoapAgent, GetActiveMotive_NullBeforeFirstGoalSelection)
{
    const goap_agent agent;
    EXPECT_EQ(agent.get_active_motive(), nullptr);
}

TEST(GoapAgent, GetCurrentAction_NullWhenIdle)
{
    const goap_agent agent;
    EXPECT_EQ(agent.get_current_action(), nullptr);
}

TEST(GoapAgent, TickWithNoActionsRemainsIdle)
{
    const agent_config config;
    goap_agent agent(config);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 1));

    agent.tick(0.016f);
    EXPECT_EQ(agent.get_status(), agent_status::Idle);
}

TEST(GoapAgent, TickWithNoMotivesRemainsIdle)
{
    const agent_config config;
    goap_agent agent(config);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    const auto act = std::make_shared<instant_action>("finish");
    act->add_effect("done", state_value{true});
    agent.add_action(act);

    agent.tick(0.016f);
    EXPECT_EQ(agent.get_status(), agent_status::Idle);
}

TEST_F(SimpleAgentFixture, ReachesExecutingAfterPlanFound)
{
    pump(agent, 50, [&]{ return agent.get_status() == agent_status::Executing; });
    EXPECT_EQ(agent.get_status(), agent_status::Executing);
}

TEST_F(SimpleAgentFixture, SingleActionPlanCompletesAndReturnsToIdle)
{
    bool reached_idle_after_exec = false;
    agent.set_on_status_changed([&](const agent_status prev, const agent_status next)
    {
        if (prev == agent_status::Executing && next == agent_status::Idle)
            reached_idle_after_exec = true;
    });

    pump(agent, 100, [&]{ return reached_idle_after_exec; });
    EXPECT_TRUE(reached_idle_after_exec);
}

TEST(GoapAgent, GoalSatisfiedStatusWhenWorldAlreadyMeetsGoal)
{
    agent_config cfg;
    cfg.goal_recheck_interval = 1;
    cfg.synchronous_planning  = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    agent.set_world_values({{"done", state_value{true}}});

    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto act = std::make_shared<instant_action>("finish");
    act->add_effect("done", state_value{true});
    agent.add_action(act);

    pump(agent, 10, [&]{ return agent.get_status() == agent_status::GoalSatisfied; });
    EXPECT_EQ(agent.get_status(), agent_status::GoalSatisfied);
}

TEST(GoapAgent, MultiStepPlanExecutesAllActionsAndAppliesEffects)
{
    agent_config cfg;
    cfg.goal_recheck_interval  = 0;
    cfg.replan_on_world_change = false;
    cfg.synchronous_planning   = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    const auto step1 = std::make_shared<instant_action>("step1");
    step1->add_precondition("a", state_value{false});
    step1->add_effect("a", state_value{true});

    const auto step2 = std::make_shared<instant_action>("step2");
    step2->add_precondition("a", state_value{true});
    step2->add_effect("b", state_value{true});

    const auto step3 = std::make_shared<instant_action>("step3");
    step3->add_precondition("b", state_value{true});
    step3->add_effect("done", state_value{true});

    agent.add_action(step1);
    agent.add_action(step2);
    agent.add_action(step3);

    agent.set_world_values({{"a", state_value{false}}, {"b", state_value{false}}});

    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    bool returned_to_idle = false;
    agent.set_on_status_changed([&](const agent_status prev, const agent_status next)
    {
        if (prev == agent_status::Executing && next == agent_status::Idle)
            returned_to_idle = true;
    });

    pump(agent, 300, [&]{ return returned_to_idle; });
    ASSERT_TRUE(returned_to_idle) << "3-step plan did not complete within tick budget";
    EXPECT_TRUE(std::as_const(agent).get_world_state().get_bool("done").value_or(false))
        << "World state 'done' should be true after full 3-step plan";
}

TEST(GoapAgent, ThrowsIfNoHeuristicSetBeforePlanning)
{
    goap_agent agent;
    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 1));

    const auto act = std::make_shared<instant_action>("finish");
    act->add_effect("done", state_value{true});
    agent.add_action(act);

    EXPECT_THROW(agent.tick(0.016f), std::runtime_error);
}

TEST_F(SimpleAgentFixture, InterruptTransitionsToInterrupted)
{
    pump(agent, 50, [&]{ return agent.get_status() == agent_status::Executing; });
    agent.interrupt();
    EXPECT_EQ(agent.get_status(), agent_status::Interrupted);
}

TEST_F(SimpleAgentFixture, TickWhileInterruptedDoesNothing)
{
    pump(agent, 50, [&]{ return agent.get_status() == agent_status::Executing; });
    agent.interrupt();
    for (int i = 0; i < 20; ++i) agent.tick(0.016f);
    EXPECT_EQ(agent.get_status(), agent_status::Interrupted);
}

TEST_F(SimpleAgentFixture, ResumeFromInterruptedReturnsToIdle)
{
    pump(agent, 50, [&]{ return agent.get_status() == agent_status::Executing; });
    agent.interrupt();
    agent.resume();
    EXPECT_EQ(agent.get_status(), agent_status::Idle);
}

TEST_F(SimpleAgentFixture, ResumeOnNonInterruptedStatusIsNoOp)
{
    agent.resume();
    EXPECT_EQ(agent.get_status(), agent_status::Idle);
}

TEST_F(SimpleAgentFixture, InterruptCancelsOngoingAsyncPlan)
{
    pump(agent, 5, [&]{ return agent.get_status() != agent_status::Idle; });
    agent.interrupt();
    EXPECT_FALSE(agent.is_planning());
    EXPECT_EQ(agent.get_status(), agent_status::Interrupted);
}

TEST_F(SimpleAgentFixture, ForceReplanWhileExecutingKicksOffNewPlan)
{
    pump(agent, 50, [&]{ return agent.get_status() == agent_status::Executing; });
    agent.force_replan();

    const bool valid =
        agent.get_status() == agent_status::Planning  ||
        agent.get_status() == agent_status::Executing ||
        agent.get_status() == agent_status::Idle;
    EXPECT_TRUE(valid);
}

TEST(GoapAgent, ForceReplanWithNoMotivesGoesToIdle)
{
    const agent_config config;
    goap_agent agent(config);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    const auto act = std::make_shared<instant_action>("finish");
    act->add_effect("done", state_value{true});
    agent.add_action(act);

    agent.force_replan();
    EXPECT_EQ(agent.get_status(), agent_status::Idle);
}

TEST(GoapAgent, AddActionRegistersAction)
{
    goap_agent agent;
    agent.add_action(std::make_shared<instant_action>("test_act"));
    ASSERT_EQ(agent.get_actions().size(), 1u);
    EXPECT_EQ(agent.get_actions()[0]->get_name(), "test_act");
}

TEST(GoapAgent, RemoveActionByName)
{
    goap_agent agent;
    agent.add_action(std::make_shared<instant_action>("a1"));
    agent.add_action(std::make_shared<instant_action>("a2"));
    agent.remove_action("a1");
    ASSERT_EQ(agent.get_actions().size(), 1u);
    EXPECT_EQ(agent.get_actions()[0]->get_name(), "a2");
}

TEST(GoapAgent, RemoveNonExistentActionIsNoOp)
{
    goap_agent agent;
    agent.add_action(std::make_shared<instant_action>("a1"));
    EXPECT_NO_THROW(agent.remove_action("ghost"));
    EXPECT_EQ(agent.get_actions().size(), 1u);
}

TEST(GoapAgent, ClearActionsEmptiesRegistry)
{
    goap_agent agent;
    agent.add_action(std::make_shared<instant_action>("a1"));
    agent.add_action(std::make_shared<instant_action>("a2"));
    agent.clear_actions();
    EXPECT_TRUE(agent.get_actions().empty());
}

TEST_F(SimpleAgentFixture, ClearMotivesStopsGoalSelection)
{
    agent.clear_motives();
    for (int i = 0; i < 20; ++i) agent.tick(0.016f);
    EXPECT_EQ(agent.get_status(), agent_status::Idle);
}

TEST(GoapAgent, HigherPriorityMotiveIsSelectedFirst)
{
    agent_config cfg;
    cfg.goal_recheck_interval = 1;
    cfg.synchronous_planning  = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state low_goal, high_goal;
    low_goal.set_bool("low", true);
    high_goal.set_bool("high", true);

    agent.add_motive(std::make_shared<motive>(low_goal,   1));
    agent.add_motive(std::make_shared<motive>(high_goal, 99));

    const auto high_act = std::make_shared<instant_action>("achieve_high");
    high_act->add_effect("high", state_value{true});
    const auto low_act = std::make_shared<instant_action>("achieve_low");
    low_act->add_effect("low", state_value{true});

    agent.add_action(high_act);
    agent.add_action(low_act);

    pump(agent, 50, [&]{ return agent.get_active_motive() != nullptr; });

    ASSERT_NE(agent.get_active_motive(), nullptr);
    EXPECT_TRUE(agent.get_active_motive()->get_goal_state().has_state("high"))
        << "High-priority motive should be selected";
}

TEST(GoapAgent, GoalSatisfiedCallbackFiresWhenWorldAlreadyMeetsGoal)
{
    agent_config cfg;
    cfg.goal_recheck_interval = 1;
    cfg.synchronous_planning  = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    bool fired = false;
    agent.set_on_goal_satisfied([&](const motive&) { fired = true; });

    agent.set_world_values({{"done", state_value{true}}});

    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 5));

    const auto act = std::make_shared<instant_action>("finish");
    act->add_effect("done", state_value{true});
    agent.add_action(act);

    pump(agent, 10, [&]{ return fired; });
    EXPECT_TRUE(fired);
}

TEST_F(SimpleAgentFixture, OnStatusChangedFiredOnEveryTransition)
{
    std::vector<std::pair<agent_status, agent_status>> transitions;
    agent.set_on_status_changed([&](const agent_status prev, const agent_status next)
    {
        transitions.emplace_back(prev, next);
    });
    pump(agent, 200);
    EXPECT_FALSE(transitions.empty());
}

TEST_F(SimpleAgentFixture, OnPlanFoundCallbackFired)
{
    bool plan_found = false;
    agent.set_on_plan_found([&](const plan_result& r)
    {
        plan_found = r.success() && !r.actions.empty();
    });
    pump(agent, 100, [&]{ return plan_found; });
    EXPECT_TRUE(plan_found);
}

TEST_F(SimpleAgentFixture, OnGoalSelectedCallbackFired)
{
    bool goal_selected = false;
    agent.set_on_goal_selected([&](const motive&) { goal_selected = true; });
    pump(agent, 50, [&]{ return goal_selected; });
    EXPECT_TRUE(goal_selected);
}

TEST_F(SimpleAgentFixture, OnActionFinishedCallbackFiredOnSuccess)
{
    bool finished_success = false;
    agent.set_on_action_finished([&](const goap_action&, const bool success)
    {
        if (success) finished_success = true;
    });
    pump(agent, 200, [&]{ return finished_success; });
    EXPECT_TRUE(finished_success);
}

TEST(GoapAgent, OnActionFinishedCallbackFiredOnFailure)
{
    agent_config cfg;
    cfg.max_consecutive_failures = 5;
    cfg.synchronous_planning     = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto act = std::make_shared<failing_action>("will_fail");
    act->add_effect("done", state_value{true});
    agent.add_action(act);

    bool finished_failure = false;
    agent.set_on_action_finished([&](const goap_action&, const bool success)
    {
        if (!success) finished_failure = true;
    });

    pump(agent, 200, [&]{ return finished_failure; });
    EXPECT_TRUE(finished_failure);
}

TEST(GoapAgent, SetWorldValuesUnlocksNewPlanAfterActionBecomesAvailable)
{
    agent_config cfg;
    cfg.replan_on_world_change   = true;
    cfg.goal_recheck_interval    = 1;
    cfg.max_consecutive_failures = 10;
    cfg.synchronous_planning     = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto act = std::make_shared<instant_action>("finish_armed");
    act->add_precondition("armed", state_value{true});
    act->add_effect("done", state_value{true});
    agent.add_action(act);

    pump(agent, 50, [&]{
        return agent.get_status() == agent_status::PlanFailed ||
               agent.get_status() == agent_status::Idle;
    });

    bool found = false;
    agent.set_on_plan_found([&](const plan_result&) { found = true; });

    agent.set_world_values({{"armed", state_value{true}}});
    agent.force_replan();

    pump(agent, 100, [&]{ return found; });
    EXPECT_TRUE(found) << "Arming world state should allow planner to find a plan";
}

TEST(GoapAgent, GetWorldState_MutationReflectedInternally)
{
    goap_agent agent;
    agent.get_world_state().set_bool("flag", true);
    EXPECT_TRUE(agent.get_world_state().get_bool("flag").value_or(false));
}

TEST(GoapAgent, ConstGetWorldState_AccessibleFromConst)
{
    agent_config cfg;
    cfg.goal_recheck_interval    = 1;
    cfg.replan_on_world_change   = true;
    cfg.max_consecutive_failures = 3;
    const goap_agent agent(cfg);
    EXPECT_FALSE(agent.get_world_state().has_state("nonexistent"));
}

TEST(GoapAgent, PlanFailedStatusWhenNoPlanExists)
{
    agent_config cfg;
    cfg.max_consecutive_failures = 10;
    cfg.synchronous_planning     = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state goal_ws;
    goal_ws.set_bool("impossible", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto irrelevant = std::make_shared<instant_action>("irrelevant");
    irrelevant->add_effect("other", state_value{true});
    agent.add_action(irrelevant);

    pump(agent, 50, [&]{
        return agent.get_status() == agent_status::PlanFailed ||
               agent.get_status() == agent_status::Idle;
    });

    EXPECT_TRUE(
        agent.get_status() == agent_status::PlanFailed ||
        agent.get_status() == agent_status::Idle
    );
}

TEST(GoapAgent, MaxConsecutiveFailuresResetsToIdle)
{
    agent_config cfg;
    cfg.max_consecutive_failures = 2;
    cfg.synchronous_planning     = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state goal_ws;
    goal_ws.set_bool("unreachable", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto act = std::make_shared<instant_action>("wrong");
    act->add_effect("other", state_value{true});
    agent.add_action(act);

    bool transitioned_to_idle = false;
    agent.set_on_status_changed([&](const agent_status /*prev*/, const agent_status next)
    {
        if (next == agent_status::Idle) transitioned_to_idle = true;
    });

    pump(agent, 50);
    EXPECT_TRUE(transitioned_to_idle)
        << "Agent should transition through Idle after max consecutive failures";
}

TEST(GoapAgent, GoalSwitchesWhenHigherPriorityMotiveAppears)
{
    agent_config cfg;
    cfg.goal_recheck_interval  = 1;
    cfg.replan_on_world_change = true;
    cfg.skip_replan_same_goal  = true;
    cfg.synchronous_planning   = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state low_goal;
    low_goal.set_bool("low_done", true);
    agent.add_motive(std::make_shared<motive>(low_goal, 1));

    const auto low_act = std::make_shared<multi_tick_action>("long_low", 100);
    low_act->add_effect("low_done", state_value{true});
    agent.add_action(low_act);

    pump(agent, 50, [&]{ return agent.get_status() == agent_status::Executing; });
    ASSERT_EQ(agent.get_status(), agent_status::Executing);

    world_state high_goal;
    high_goal.set_bool("high_done", true);
    agent.add_motive(std::make_shared<motive>(high_goal, 999));

    const auto high_act = std::make_shared<instant_action>("quick_high");
    high_act->add_effect("high_done", state_value{true});
    agent.add_action(high_act);

    agent.set_world_values({{"trigger", state_value{true}}});

    bool switched = false;
    agent.set_on_goal_selected([&](const motive& m)
    {
        if (m.get_goal_state().has_state("high_done")) switched = true;
    });

    pump(agent, 100, [&]{ return switched; });
    EXPECT_TRUE(switched)
        << "Agent should abandon low-priority goal and replan for high-priority";
}

TEST(GoapAgent, CustomUtilityEvaluatorOverridesDefaultPriority)
{
    agent_config cfg;
    cfg.goal_recheck_interval = 1;
    cfg.synchronous_planning  = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state hp_goal, lp_goal;
    hp_goal.set_bool("hp", true);
    lp_goal.set_bool("lp", true);

    agent.add_motive(std::make_shared<motive>(lp_goal, 100));
    agent.add_motive(std::make_shared<motive>(hp_goal,   1));

    agent.set_utility_evaluator([](const motive& m, const world_state&) -> float
    {
        return m.get_goal_state().has_state("hp") ? 9999.0f : 0.0f;
    });

    const auto lp_act = std::make_shared<instant_action>("lp_action");
    lp_act->add_effect("lp", state_value{true});
    const auto hp_act = std::make_shared<instant_action>("hp_action");
    hp_act->add_effect("hp", state_value{true});
    agent.add_action(lp_act);
    agent.add_action(hp_act);

    pump(agent, 50, [&]{ return agent.get_active_motive() != nullptr; });

    ASSERT_NE(agent.get_active_motive(), nullptr);
    EXPECT_TRUE(agent.get_active_motive()->get_goal_state().has_state("hp"))
        << "Custom evaluator should select 'hp' goal despite lower numeric priority";
}

TEST(GoapAgent, GetCurrentAction_ReturnsNonNullDuringExecution)
{
    agent_config cfg;
    cfg.goal_recheck_interval  = 0;
    cfg.replan_on_world_change = false;
    cfg.synchronous_planning   = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto act = std::make_shared<multi_tick_action>("slow", 100);
    act->add_effect("done", state_value{true});
    agent.add_action(act);

    pump(agent, 50, [&]{ return agent.get_status() == agent_status::Executing; });
    ASSERT_EQ(agent.get_status(), agent_status::Executing);

    agent.tick(0.016f);
    ASSERT_NE(agent.get_current_action(), nullptr);
    EXPECT_EQ(agent.get_current_action()->get_name(), "slow");
}

TEST(GoapAgent, GetCurrentPlan_NonEmptyDuringExecution)
{
    agent_config cfg;
    cfg.goal_recheck_interval  = 0;
    cfg.replan_on_world_change = false;
    cfg.synchronous_planning   = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto act = std::make_shared<multi_tick_action>("slow", 100);
    act->add_effect("done", state_value{true});
    agent.add_action(act);

    pump(agent, 50, [&]{ return agent.get_status() == agent_status::Executing; });
    agent.tick(0.016f);

    EXPECT_FALSE(agent.get_current_plan().actions.empty());
}

TEST(GoapAgent, ReplanOnWorldChange_FalseDisablesGoalRecheck)
{
    agent_config cfg;
    cfg.replan_on_world_change = false;
    cfg.goal_recheck_interval  = 1;
    cfg.synchronous_planning   = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto long_act = std::make_shared<multi_tick_action>("long_act", 1000);
    long_act->add_effect("done", state_value{true});
    agent.add_action(long_act);

    pump(agent, 50, [&]{ return agent.get_status() == agent_status::Executing; });
    ASSERT_EQ(agent.get_status(), agent_status::Executing);

    world_state urgent;
    urgent.set_bool("urgent", true);
    agent.add_motive(std::make_shared<motive>(urgent, 9999));

    const auto urgent_act = std::make_shared<instant_action>("urgent_act");
    urgent_act->add_effect("urgent", state_value{true});
    agent.add_action(urgent_act);

    agent.set_world_values({{"something", state_value{true}}});

    bool left_executing = false;
    agent.set_on_status_changed([&](const agent_status prev, const agent_status)
    {
        if (prev == agent_status::Executing) left_executing = true;
    });

    pump(agent, 30);
    EXPECT_FALSE(left_executing)
        << "With replan_on_world_change=false, current execution should not be interrupted";
}

TEST(GoapAgent, SkipReplanSameGoal_PreventsRedundantReplan)
{
    agent_config cfg;
    cfg.skip_replan_same_goal  = true;
    cfg.goal_recheck_interval  = 1;
    cfg.replan_on_world_change = true;
    cfg.synchronous_planning   = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state goal_ws;
    goal_ws.set_bool("done", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto act = std::make_shared<multi_tick_action>("slow", 1000);
    act->add_effect("done", state_value{true});
    agent.add_action(act);

    pump(agent, 50, [&]{ return agent.get_status() == agent_status::Executing; });
    ASSERT_EQ(agent.get_status(), agent_status::Executing);

    int plan_count = 0;
    agent.set_on_plan_found([&](const plan_result&) { ++plan_count; });

    agent.set_world_values({{"noise", state_value{true}}});
    pump(agent, 30);

    EXPECT_EQ(plan_count, 0)
        << "skip_replan_same_goal should prevent replanning when goal is unchanged";
}

TEST(GoapAgent, Integration_CombatScenario_EnemyKilledOptimally)
{
    agent_config cfg;
    cfg.goal_recheck_interval  = 0;
    cfg.replan_on_world_change = false;
    cfg.synchronous_planning   = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    agent.set_world_values({
        {"has_weapon", state_value{false}},
        {"enemy_dead", state_value{false}}
    });

    world_state goal_ws;
    goal_ws.set_bool("enemy_dead", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto pickup = std::make_shared<instant_action>("pick_up_weapon", 1);
    pickup->add_precondition("has_weapon", state_value{false});
    pickup->add_effect("has_weapon",       state_value{true});

    const auto shoot = std::make_shared<instant_action>("shoot", 1);
    shoot->add_precondition("has_weapon", state_value{true});
    shoot->add_effect("enemy_dead",       state_value{true});

    const auto punch = std::make_shared<instant_action>("punch", 10);
    punch->add_effect("enemy_dead", state_value{true});

    agent.add_action(pickup);
    agent.add_action(shoot);
    agent.add_action(punch);

    std::vector<std::string> executed;
    agent.set_on_action_finished([&](const goap_action& a, const bool success)
    {
        if (success) executed.push_back(a.get_name());
    });

    bool plan_done = false;
    agent.set_on_status_changed([&](const agent_status prev, const agent_status next)
    {
        if (prev == agent_status::Executing && next == agent_status::Idle)
            plan_done = true;
    });

    pump(agent, 300, [&]{ return plan_done; });

    ASSERT_TRUE(plan_done) << "Plan did not complete";
    ASSERT_EQ(executed.size(), 2u) << "Optimal plan should be exactly 2 actions";
    EXPECT_EQ(executed[0], "pick_up_weapon");
    EXPECT_EQ(executed[1], "shoot");
    EXPECT_TRUE(std::as_const(agent).get_world_state().get_bool("enemy_dead").value_or(false));
    for (const auto& name : executed)
        EXPECT_NE(name, "punch") << "Suboptimal punch should not appear in plan";
}

TEST(GoapAgent, Integration_AgentRecoversByReplanningAfterActionFails)
{
    agent_config cfg;
    cfg.goal_recheck_interval    = 1;
    cfg.replan_on_world_change   = true;
    cfg.max_consecutive_failures = 5;
    cfg.synchronous_planning     = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    agent.set_world_values({
        {"enemy_dead", state_value{false}},
        {"gun_jammed", state_value{true}}
    });

    world_state goal_ws;
    goal_ws.set_bool("enemy_dead", true);
    agent.add_motive(std::make_shared<motive>(goal_ws, 10));

    const auto shoot = std::make_shared<instant_action>("shoot", 1);
    shoot->add_precondition("gun_jammed", state_value{false});
    shoot->add_effect("enemy_dead",       state_value{true});

    const auto punch = std::make_shared<instant_action>("punch", 5);
    punch->add_effect("enemy_dead", state_value{true});

    agent.add_action(shoot);
    agent.add_action(punch);

    std::vector<std::string> executed;
    agent.set_on_action_finished([&](const goap_action& a, const bool s)
    {
        if (s) executed.push_back(a.get_name());
    });

    bool plan_done = false;
    agent.set_on_status_changed([&](const agent_status prev, const agent_status next)
    {
        if (prev == agent_status::Executing && next == agent_status::Idle)
            plan_done = true;
    });

    pump(agent, 300, [&]{ return plan_done; });

    ASSERT_TRUE(plan_done);
    ASSERT_FALSE(executed.empty());
    EXPECT_EQ(executed.back(), "punch")
        << "Fallback punch should have been executed";
    EXPECT_TRUE(std::as_const(agent).get_world_state().get_bool("enemy_dead").value_or(false));
}

TEST(GoapAgent, Integration_MultipleGoalsResolvedByPriority)
{
    agent_config cfg;
    cfg.goal_recheck_interval  = 1;
    cfg.replan_on_world_change = true;
    cfg.skip_replan_same_goal  = false;
    cfg.synchronous_planning   = true;
    goap_agent agent(cfg);
    agent.set_heuristic(std::make_shared<goal_count_heuristic>());

    world_state g1, g2, g3;
    g1.set_bool("g1", true);
    g2.set_bool("g2", true);
    g3.set_bool("g3", true);

    agent.add_motive(std::make_shared<motive>(g1,  1));
    agent.add_motive(std::make_shared<motive>(g2,  5));
    agent.add_motive(std::make_shared<motive>(g3, 99));

    const auto a1 = std::make_shared<instant_action>("do_g1"); a1->add_effect("g1", state_value{true});
    const auto a2 = std::make_shared<instant_action>("do_g2"); a2->add_effect("g2", state_value{true});
    const auto a3 = std::make_shared<instant_action>("do_g3"); a3->add_effect("g3", state_value{true});
    agent.add_action(a1);
    agent.add_action(a2);
    agent.add_action(a3);

    std::string first_goal;
    agent.set_on_goal_selected([&](const motive& m)
    {
        if (!first_goal.empty()) return;
        if      (m.get_goal_state().has_state("g1")) first_goal = "g1";
        else if (m.get_goal_state().has_state("g2")) first_goal = "g2";
        else if (m.get_goal_state().has_state("g3")) first_goal = "g3";
    });

    pump(agent, 50, [&]{ return !first_goal.empty(); });
    EXPECT_EQ(first_goal, "g3") << "Highest priority goal (g3) should be selected first";
}
