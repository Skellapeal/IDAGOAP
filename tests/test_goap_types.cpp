//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "goap_types.h"
#include "world_state.h"

using namespace rida_goap;

TEST(StateCondition, EqualBoolTrue)
{
    world_state ws;
    ws.set_bool("armed", true);
    const state_condition cond(state_value{true}, predicate_op::Equal);
    EXPECT_TRUE(cond.evaluate(ws, "armed"));
}

TEST(StateCondition, EqualBoolFalse)
{
    world_state ws;
    ws.set_bool("armed", false);
    const state_condition cond(state_value{true}, predicate_op::Equal);
    EXPECT_FALSE(cond.evaluate(ws, "armed"));
}

TEST(StateCondition, NotEqualInt)
{
    world_state ws;
    ws.set_int("ammo", 5);
    const state_condition cond(state_value{3}, predicate_op::NotEqual);
    EXPECT_TRUE(cond.evaluate(ws, "ammo"));
}

TEST(StateCondition, LessInt)
{
    world_state ws;
    ws.set_int("health", 20);
    const state_condition cond(state_value{50}, predicate_op::Less);
    EXPECT_TRUE(cond.evaluate(ws, "health"));
}

TEST(StateCondition, LessOrEqualIntExact)
{
    world_state ws;
    ws.set_int("health", 50);
    const state_condition cond(state_value{50}, predicate_op::LessOrEqual);
    EXPECT_TRUE(cond.evaluate(ws, "health"));
}

TEST(StateCondition, GreaterInt)
{
    world_state ws;
    ws.set_int("ammo", 10);
    const state_condition cond(state_value{5}, predicate_op::Greater);
    EXPECT_TRUE(cond.evaluate(ws, "ammo"));
}

TEST(StateCondition, GreaterOrEqualFloat)
{
    world_state ws;
    ws.set_float("speed", 3.0f);
    const state_condition cond(state_value{3.0f}, predicate_op::GreaterOrEqual);
    EXPECT_TRUE(cond.evaluate(ws, "speed"));
}

TEST(StateCondition, BoolWithLessPredicateReturnsFalse)
{
    world_state ws;
    ws.set_bool("flag", true);
    const state_condition cond(state_value{true}, predicate_op::Less);
    EXPECT_FALSE(cond.evaluate(ws, "flag"));
}

TEST(StateCondition, StringWithGreaterPredicateReturnsFalse)
{
    world_state ws;
    ws.set_string("mode", "attack");
    const state_condition cond(state_value{std::string("attack")}, predicate_op::Greater);
    EXPECT_FALSE(cond.evaluate(ws, "mode"));
}

TEST(StateCondition, EvaluateOnMissingKeyReturnsFalse)
{
    const world_state ws;
    const state_condition cond(state_value{true}, predicate_op::Equal);
    EXPECT_FALSE(cond.evaluate(ws, "nonexistent"));
}

TEST(StateCondition, EvaluateTypeMismatchReturnsFalse)
{
    world_state ws;
    ws.set_int("x", 1);
    const state_condition cond(state_value{true}, predicate_op::Equal);
    EXPECT_FALSE(cond.evaluate(ws, "x"));
}

TEST(StateValueHash, SameIntHashesEqual)
{
    const std::hash<state_value> h;
    EXPECT_EQ(h(state_value{42}), h(state_value{42}));
}

TEST(StateValueHash, DifferentIntHashesDiffer)
{
    const std::hash<state_value> h;
    EXPECT_NE(h(state_value{1}), h(state_value{2}));
}

TEST(StateValueHash, VectorFloatHashesDeterministic)
{
    const std::hash<state_value> h;
    const state_value a{std::vector{1.0f, 2.0f}};
    const state_value b{std::vector{1.0f, 2.0f}};
    EXPECT_EQ(h(a), h(b));
}
