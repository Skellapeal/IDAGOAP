//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "world_state.h"

using namespace rida_goap;

TEST(WorldState, SetAndGetBool)
{
    world_state ws;
    ws.set_bool("alive", true);
    ASSERT_TRUE(ws.get_bool("alive").has_value());
    EXPECT_TRUE(*ws.get_bool("alive"));
}

TEST(WorldState, SetAndGetInt)
{
    world_state ws;
    ws.set_int("ammo", 10);
    ASSERT_TRUE(ws.get_int("ammo").has_value());
    EXPECT_EQ(*ws.get_int("ammo"), 10);
}

TEST(WorldState, SetAndGetFloat)
{
    world_state ws;
    ws.set_float("health", 75.5f);
    ASSERT_TRUE(ws.get_float("health").has_value());
    EXPECT_FLOAT_EQ(*ws.get_float("health"), 75.5f);
}

TEST(WorldState, SetAndGetString)
{
    world_state ws;
    ws.set_string("mode", "patrol");
    ASSERT_TRUE(ws.get_string("mode").has_value());
    EXPECT_EQ(*ws.get_string("mode"), "patrol");
}

TEST(WorldState, SetAndGetPosition)
{
    world_state ws;
    ws.set_position("pos", 1.0f, 2.0f, 3.0f);
    const auto p = ws.get_position("pos");
    ASSERT_TRUE(p.has_value());
    EXPECT_FLOAT_EQ((*p)[0], 1.0f);
    EXPECT_FLOAT_EQ((*p)[1], 2.0f);
    EXPECT_FLOAT_EQ((*p)[2], 3.0f);
}

TEST(WorldState, HasStateReturnsTrueForExistingKey)
{
    world_state ws;
    ws.set_bool("armed", false);
    EXPECT_TRUE(ws.has_state("armed"));
}

TEST(WorldState, HasStateReturnsFalseForMissingKey)
{
    const world_state ws;
    EXPECT_FALSE(ws.has_state("nonexistent"));
}

TEST(WorldState, RemoveStateErasesKey)
{
    world_state ws;
    ws.set_bool("flag", true);
    ws.remove_state("flag");
    EXPECT_FALSE(ws.has_state("flag"));
}

TEST(WorldState, RemoveNonExistentKeyDoesNotThrow)
{
    world_state ws;
    EXPECT_NO_THROW(ws.remove_state("ghost"));
}

TEST(WorldState, OverwriteIntValue)
{
    world_state ws;
    ws.set_int("count", 1);
    ws.set_int("count", 99);
    EXPECT_EQ(*ws.get_int("count"), 99);
}

TEST(WorldState, GetBoolOnIntKeyReturnsNullopt)
{
    world_state ws;
    ws.set_int("x", 5);
    EXPECT_FALSE(ws.get_bool("x").has_value());
}

TEST(WorldState, GetIntOnMissingKeyReturnsNullopt)
{
    const world_state ws;
    EXPECT_FALSE(ws.get_int("missing").has_value());
}

TEST(WorldState, MergeAddsKeysFromOther)
{
    world_state a, b;
    a.set_bool("alive", true);
    b.set_int("ammo", 5);
    a.merge(b);
    EXPECT_TRUE(a.has_state("alive"));
    EXPECT_TRUE(a.has_state("ammo"));
}

TEST(WorldState, MergeOverwritesExistingKey)
{
    world_state a, b;
    a.set_int("ammo", 10);
    b.set_int("ammo", 3);
    a.merge(b);
    EXPECT_EQ(*a.get_int("ammo"), 3);
}

TEST(WorldState, MergeOverwritesWithDifferentType)
{
    world_state a, b;
    a.set_bool("x", true);
    b.set_int("x", 99);
    a.merge(b);
    EXPECT_FALSE(a.get_bool("x").has_value());
    ASSERT_TRUE(a.get_int("x").has_value());
    EXPECT_EQ(*a.get_int("x"), 99);
}

TEST(WorldState, MergeWithEmptyOtherIsNoOp)
{
    world_state a;
    const world_state b;
    a.set_bool("alive", true);
    a.merge(b);
    EXPECT_EQ(a.get_states().size(), 1u);
}

TEST(WorldState, SatisfiesReturnsTrueWhenAllGoalKeysMatch)
{
    world_state world, goal;
    world.set_bool("alive", true);
    world.set_int("ammo", 5);
    goal.set_bool("alive", true);
    EXPECT_TRUE(world.satisfies(goal));
}

TEST(WorldState, SatisfiesReturnsFalseWhenValueMismatch)
{
    world_state world, goal;
    world.set_bool("alive", false);
    goal.set_bool("alive", true);
    EXPECT_FALSE(world.satisfies(goal));
}

TEST(WorldState, SatisfiesReturnsFalseWhenKeyMissing)
{
    const world_state world;
    world_state goal;
    goal.set_bool("alive", true);
    EXPECT_FALSE(world.satisfies(goal));
}

TEST(WorldState, SatisfiesReturnsFalseOnTypeMismatch)
{
    world_state ws_int, ws_bool;
    ws_int .set_int ("flag", 1);
    ws_bool.set_bool("flag", true);

    world_state goal_bool, goal_int;
    goal_bool.set_bool("flag", true);
    goal_int .set_int ("flag", 1);

    EXPECT_FALSE(ws_int .satisfies(goal_bool));
    EXPECT_FALSE(ws_bool.satisfies(goal_int ));
}

TEST(WorldState, SatisfiesReturnsTrueForEmptyGoal)
{
    world_state world;
    const world_state goal;
    world.set_bool("alive", true);
    EXPECT_TRUE(world.satisfies(goal));
}

TEST(WorldState, EqualityTrueForIdenticalStates)
{
    world_state a, b;
    a.set_bool("x", true);
    b.set_bool("x", true);
    EXPECT_EQ(a, b);
}

TEST(WorldState, EqualityFalseForDifferentValues)
{
    world_state a, b;
    a.set_int("x", 1);
    b.set_int("x", 2);
    EXPECT_NE(a, b);
}

TEST(WorldState, EqualityFalseForDifferentKeys)
{
    world_state a, b;
    a.set_bool("x", true);
    b.set_bool("y", true);
    EXPECT_NE(a, b);
}

TEST(WorldState, HashIsDeterministicForSameState)
{
    world_state a, b;
    a.set_bool("alive", true);
    a.set_int("ammo", 3);
    b.set_bool("alive", true);
    b.set_int("ammo", 3);
    const std::hash<world_state> h;
    EXPECT_EQ(h(a), h(b));
}

TEST(WorldState, HashDiffersForDifferentStates)
{
    world_state a, b;
    a.set_int("ammo", 3);
    b.set_int("ammo", 4);
    EXPECT_NE(std::hash<world_state>{}(a), std::hash<world_state>{}(b));
}

TEST(WorldState, GetIntOnBoolKeyReturnsNullopt)
{
    world_state ws;
    ws.set_bool("flag", true);
    EXPECT_FALSE(ws.get_int("flag").has_value());
}

TEST(WorldState, GetFloatOnStringKeyReturnsNullopt)
{
    world_state ws;
    ws.set_string("mode", "attack");
    EXPECT_FALSE(ws.get_float("mode").has_value());
}

TEST(WorldState, GetStringOnFloatKeyReturnsNullopt)
{
    world_state ws;
    ws.set_float("speed", 1.5f);
    EXPECT_FALSE(ws.get_string("speed").has_value());
}

TEST(WorldState, GetPositionOnBoolKeyReturnsNullopt)
{
    world_state ws;
    ws.set_bool("at_base", true);
    EXPECT_FALSE(ws.get_position("at_base").has_value());
}

TEST(WorldState, SetPosition2DStoresTwoComponents)
{
    world_state ws;
    ws.set_position_2d("pos", 4.0f, 7.0f);
    const auto p = ws.get_position("pos");
    ASSERT_TRUE(p.has_value());
    ASSERT_EQ(p->size(), 2u);
    EXPECT_FLOAT_EQ((*p)[0], 4.0f);
    EXPECT_FLOAT_EQ((*p)[1], 7.0f);
}

TEST(WorldState, MergeDefaultsDoesNotOverwriteExistingKey)
{
    world_state a, b;
    a.set_int("ammo", 10);
    b.set_int("ammo", 99);
    a.merge_defaults(b);
    EXPECT_EQ(*a.get_int("ammo"), 10);
}

TEST(WorldState, MergeDefaultsAddsAbsentKeys)
{
    world_state a, b;
    a.set_bool("alive", true);
    b.set_int("ammo", 5);
    a.merge_defaults(b);
    ASSERT_TRUE(a.get_int("ammo").has_value());
    EXPECT_EQ(*a.get_int("ammo"), 5);
}

TEST(WorldState, MergeDefaultsWithEmptySourceIsNoOp)
{
    world_state a;
    const world_state b;
    a.set_bool("alive", true);
    a.merge_defaults(b);
    EXPECT_EQ(a.get_states().size(), 1u);
}

TEST(WorldState, SatisfiesConditionMapReturnsTrueWhenAllConditionsMet)
{
    world_state ws;
    ws.set_int("ammo", 10);

    std::unordered_map<std::string, state_condition> conditions;
    conditions["ammo"] = state_condition(state_value{5}, predicate_op::Greater);

    EXPECT_TRUE(ws.satisfies(conditions));
}

TEST(WorldState, SatisfiesConditionMapReturnsFalseWhenConditionNotMet)
{
    world_state ws;
    ws.set_int("ammo", 2);

    std::unordered_map<std::string, state_condition> conditions;
    conditions["ammo"] = state_condition(state_value{5}, predicate_op::Greater);

    EXPECT_FALSE(ws.satisfies(conditions));
}

TEST(WorldState, SatisfiesEmptyConditionMapAlwaysReturnsTrue)
{
    world_state ws;
    ws.set_bool("alive", true);

    const std::unordered_map<std::string, state_condition> conditions;
    EXPECT_TRUE(ws.satisfies(conditions));
}

TEST(WorldState, SatisfiesConditionMapReturnsFalseForMissingKey)
{
    const world_state ws;

    std::unordered_map<std::string, state_condition> conditions;
    conditions["ammo"] = state_condition(state_value{0}, predicate_op::Equal);

    EXPECT_FALSE(ws.satisfies(conditions));
}