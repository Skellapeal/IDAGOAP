//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "goap_action.h"
#include "world_state.h"

using namespace rida_goap;

class test_action : public goap_action
{
public:
    bool started = false;
    bool ended = false;
    bool interrupted = false;
    bool can_run_flag = true;

    test_action(const std::string& name, const int cost) : goap_action(name, cost) {}

    bool on_start() override { started = true; return true; }
    action_status on_tick(float) override { return action_status::Succeeded; }
    void on_end(bool) override { ended = true; }
    void on_interrupt() override { interrupted = true; }
    bool can_run() const override { return can_run_flag; }
};

class bare_action : public goap_action
{
public:
    bare_action(const std::string& name, const int cost) : goap_action(name, cost) {}
    action_status on_tick(float) override { return action_status::Succeeded; }
};

TEST(GoapAction, NameAndCostAreSet)
{
    const test_action a("attack", 3);
    EXPECT_EQ(a.get_name(), "attack");
    EXPECT_EQ(a.get_cost(), 3);
}

TEST(GoapAction, AddAndCheckPreconditionMet)
{
    test_action a("shoot", 1);
    a.add_precondition("has_weapon", state_value{true});
    world_state ws;
    ws.set_bool("has_weapon", true);
    EXPECT_TRUE(a.check_preconditions(ws));
}

TEST(GoapAction, CheckPreconditionNotMet)
{
    test_action a("shoot", 1);
    a.add_precondition("has_weapon", state_value{true});
    world_state ws;
    ws.set_bool("has_weapon", false);
    EXPECT_FALSE(a.check_preconditions(ws));
}

TEST(GoapAction, CheckPreconditionsAllMet)
{
    test_action a("shoot", 1);
    a.add_precondition("has_weapon", state_value{true});
    a.add_precondition("ammo", state_value{5}, predicate_op::Greater);
    world_state ws;
    ws.set_bool("has_weapon", true);
    ws.set_int("ammo", 10);
    EXPECT_TRUE(a.check_preconditions(ws));
}

TEST(GoapAction, CheckPreconditionsOneMissing)
{
    test_action a("shoot", 1);
    a.add_precondition("has_weapon", state_value{true});
    a.add_precondition("ammo", state_value{5}, predicate_op::Greater);
    world_state ws;
    ws.set_bool("has_weapon", true);
    EXPECT_FALSE(a.check_preconditions(ws));
}

TEST(GoapAction, CheckPreconditionsGreaterFailsAtBoundary)
{
    test_action a("shoot", 1);
    a.add_precondition("ammo", state_value{5}, predicate_op::Greater);
    world_state ws;
    ws.set_int("ammo", 5);
    EXPECT_FALSE(a.check_preconditions(ws));
}

TEST(GoapAction, NoPreconditionsAlwaysPasses)
{
    const test_action a("idle", 0);
    const world_state ws;
    EXPECT_TRUE(a.check_preconditions(ws));
}

TEST(GoapAction, ApplyEffectSetsBool)
{
    test_action a("kill", 1);
    a.add_effect("enemy_dead", state_value{true});
    world_state ws;
    a.apply_effects(ws);
    EXPECT_TRUE(*ws.get_bool("enemy_dead"));
}

TEST(GoapAction, ApplyEffectOverwritesExistingValue)
{
    test_action a("reload", 1);
    a.add_effect("ammo", state_value{30});
    world_state ws;
    ws.set_int("ammo", 0);
    a.apply_effects(ws);
    EXPECT_EQ(*ws.get_int("ammo"), 30);
    EXPECT_FALSE(ws.get_bool("ammo").has_value());
}

TEST(GoapAction, ApplyMultipleEffects)
{
    test_action a("loot", 2);
    a.add_effect("has_weapon", state_value{true});
    a.add_effect("ammo", state_value{10});
    world_state ws;
    a.apply_effects(ws);
    EXPECT_TRUE(*ws.get_bool("has_weapon"));
    EXPECT_EQ(*ws.get_int("ammo"), 10);
}

TEST(GoapAction, CanRunDefaultIsTrue)
{
    const bare_action a("idle", 0);
    EXPECT_TRUE(a.can_run());
}

TEST(GoapAction, CanRunReturnsFalseWhenDisabled)
{
    test_action a("disabled", 0);
    a.can_run_flag = false;
    EXPECT_FALSE(a.can_run());
}

TEST(GoapAction, OnStartIsCalled)
{
    test_action a("move", 1);
    a.on_start();
    EXPECT_TRUE(a.started);
}

TEST(GoapAction, OnTickReturnSucceeded)
{
    test_action a("move", 1);
    EXPECT_EQ(a.on_tick(0.016f), action_status::Succeeded);
}

TEST(GoapAction, OnEndIsCalled)
{
    test_action a("move", 1);
    a.on_end(true);
    EXPECT_TRUE(a.ended);
}

TEST(GoapAction, OnInterruptIsCalled)
{
    test_action a("move", 1);
    a.on_interrupt();
    EXPECT_TRUE(a.interrupted);
}