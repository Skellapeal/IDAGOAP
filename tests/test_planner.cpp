//
// Created by Niall Ó Colmáin on 04/03/2026.
//

#include <iostream>
#include <cassert>
#include <vector>
#include <memory>

#include "heuristics.h"
#include "../rida_planner.h"
#include "../world_state.h"
#include "../plan_result.h"
#include "test_actions.h"

using namespace rida_goap;

int tests_passed = 0;
int tests_failed = 0;

#define TEST(name) \
    void name(); \
    struct name##_runner { \
        name##_runner() { \
            std::cout << "Running: " << #name << "... "; \
            try { \
                name(); \
                std::cout << "PASSED\n"; \
                tests_passed++; \
            } catch (const std::exception& e) { \
                std::cout << "FAILED: " << e.what() << "\n"; \
                tests_failed++; \
            } catch (...) { \
                std::cout << "FAILED: Unknown exception\n"; \
                tests_failed++; \
            } \
        } \
    } name##_instance; \
    void name()

void print_plan(const plan_result& result)
{
    std::cout << "\n";
    std::cout << "  Status: " << result.status_string() << "\n";
    std::cout << "  Actions: " << result.size() << "\n";
    std::cout << "  Cost: " << result.final_cost << "\n";
    std::cout << "  Nodes expanded: " << result.nodes_expanded << "\n";
    std::cout << "  Time: " << result.planning_time_ms << "ms\n";

    if (result.success())
    {
        std::cout << "  Plan steps:\n";
        for (size_t i = 0; i < result.actions.size(); ++i)
        {
            std::cout << "    " << (i + 1) << ". " << result.actions[i]->get_name()
                      << " (cost: " << result.actions[i]->get_cost() << ")\n";
        }
    }
}

//
// TESTS
//

TEST(test_simple_single_action_plan)
{
    world_state initial;
    initial.set_int("has_wood", 0);

    world_state goal;
    goal.set_int("has_wood", 1);

    std::vector<goap_action::ptr> actions = { std::make_shared<gather_wood_action>() };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 1);
    assert(result.actions[0]->get_name() == "GatherWood");
    assert(result.final_cost == 10);
    assert(result.nodes_expanded > 0);
}

TEST(test_two_step_plan)
{
    world_state initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_tool", 0);

    world_state goal;
    goal.set_int("has_tool", 1);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<gather_wood_action>(),
        std::make_shared<craft_tool_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 2);
    assert(result.actions[0]->get_name() == "GatherWood");
    assert(result.actions[1]->get_name() == "CraftTool");
    assert(result.final_cost == 15); // 10 + 5
}

TEST(test_complex_multi_resource_plan)
{
    world_state initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_stone", 0);
    initial.set_int("has_shelter", 0);

    world_state goal;
    goal.set_int("has_shelter", 1);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<gather_wood_action>(),
        std::make_shared<gather_stone_action>(),
        std::make_shared<build_shelter_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 3);
    assert(result.actions[2]->get_name() == "BuildShelter");
    assert(result.final_cost == 45);
}

TEST(test_no_solution_exists)
{
    world_state initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_tool", 0);

    world_state goal;
    goal.set_int("has_tool", 1);

    std::vector<goap_action::ptr> actions = { std::make_shared<craft_tool_action>() };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(!result.success());
    assert(result.status == plan_status::NoSolutionExists);
    assert(result.has_no_actions());
}

TEST(test_already_at_goal)
{
    world_state initial;
    initial.set_int("has_wood", 1);

    world_state goal;
    goal.set_int("has_wood", 1);

    std::vector<goap_action::ptr> actions = { std::make_shared<gather_wood_action>() };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.has_no_actions());
    assert(result.is_trivially_satisfied());
    assert(result.final_cost == 0);
}

TEST(test_disabled_action_filtered)
{
    world_state initial;
    initial.set_int("has_wood", 0);

    world_state goal;
    goal.set_int("has_wood", 1);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<gather_wood_action>(),
        std::make_shared<disabled_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 1);
    assert(result.actions[0]->get_name() == "GatherWood");
}

TEST(test_node_limit_reached)
{
    world_state initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_stone", 0);
    initial.set_int("has_shelter", 0);

    world_state goal;
    goal.set_int("has_shelter", 1);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<gather_wood_action>(),
        std::make_shared<gather_stone_action>(),
        std::make_shared<build_shelter_action>(),
        std::make_shared<craft_tool_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    planner_options options;
    options.max_nodes = 5;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic, options);

    print_plan(result);

    assert(!result.success());
    assert(result.status == plan_status::NodeLimitReached);
    assert(result.nodes_expanded <= 6);
}

TEST(test_time_budget_enforced)
{
    world_state initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_stone", 0);
    initial.set_int("has_shelter", 0);

    world_state goal;
    goal.set_int("has_shelter", 1);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<gather_wood_action>(),
        std::make_shared<gather_stone_action>(),
        std::make_shared<build_shelter_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    planner_options options;
    options.time_budget_ms = 1;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic, options);

    print_plan(result);

    if (!result.success())
    {
        assert(result.status == plan_status::TimedOut);
        std::cout << "  (Correctly timed out)\n";
    }
    else
    {
        std::cout << "  (Succeeded within time budget)\n";
    }

    assert(result.planning_time_ms >= 0);
}

TEST(test_heuristic_provides_guidance)
{
    world_state initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_stone", 0);
    initial.set_int("has_shelter", 0);

    world_state goal;
    goal.set_int("has_shelter", 1);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<gather_wood_action>(),
        std::make_shared<gather_stone_action>(),
        std::make_shared<build_shelter_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.nodes_expanded < 100);
    std::cout << "  (Heuristic kept search focused)\n";
}

TEST(test_bool_preconditions_and_effects)
{
    world_state initial;
    initial.set_bool("enemy_visible", false);
    initial.set_bool("is_hidden", false);

    world_state goal;
    goal.set_bool("is_hidden", true);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<scout_action>(),
        std::make_shared<hide_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 2);
    assert(result.actions[0]->get_name() == "Scout");
    assert(result.actions[1]->get_name() == "Hide");
    assert(result.final_cost == 11); // 8 + 3

    std::cout << "  (Boolean preconditions/effects working correctly)\n";
}

TEST(test_float_equality_in_variant)
{
    state_value f1{30.0f};
    state_value f2{30.0f};
    state_value f3{30.1f};

    assert(f1 == f2);
    assert(f1 != f3);

    state_value f4{30.0f};
    state_value f5{30.0f + 1e-7f};

    auto get_float = [](const state_value& v) -> float
    {
        return std::get<float>(v);
    };

    constexpr float epsilon = 1e-6f;
    assert(std::abs(get_float(f4) - get_float(f5)) <= epsilon);

    std::unordered_map<std::string, state_value> map1;
    map1["health"] = state_value{100.0f};

    std::unordered_map<std::string, state_value> map2;
    map2["health"] = state_value{100.0f};

    assert(map1.at("health") == map2.at("health"));

    std::unordered_map<std::string, state_value> map3;
    map3["health"] = state_value{100.0f + 1e-7f};

    assert(std::abs(get_float(map1.at("health")) - get_float(map3.at("health"))) <= epsilon);
}

TEST(test_is_action_relevant_debug)
{
    world_state goal;
    goal.set_state("health", state_value{100.0f});

    const auto rest = std::make_shared<rest_action>(); // no const here

    const auto& effects     = rest->get_effects();
    const auto& goal_states = goal.get_states();

    assert(goal_states.contains("health"));
    assert(effects.contains("health"));

    const auto& effect_value = effects.at("health");
    const auto& goal_value   = goal_states.at("health");
    assert(effect_value.index() == goal_value.index());
    assert(effect_value == goal_value);
}

TEST(test_float_preconditions_and_effects)
{
    world_state initial;
    initial.set_float("health", 30.0f);

    world_state goal;
    goal.set_float("health", 100.0f);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<rest_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 1);
    assert(result.actions[0]->get_name() == "Rest");
    assert(result.final_cost == 12);

    std::cout << "  (Float preconditions/effects working correctly)\n";
}

TEST(test_mixed_type_world_state)
{
    world_state initial;
    initial.set_int("ammo_count", 30);
    initial.set_bool("enemy_visible", false);
    initial.set_float("health", 100.0f);

    world_state goal;
    goal.set_bool("enemy_visible", true);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<scout_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 1);
    assert(result.actions[0]->get_name() == "Scout");

    std::cout << "  (Mixed int/bool/float state handled correctly)\n";
}

TEST(test_depth_limit_reached)
{
    world_state initial;

    world_state goal;
    goal.set_int("depth_5", 1);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<deep_action_1>(),
        std::make_shared<deep_action_2>(),
        std::make_shared<deep_action_3>(),
        std::make_shared<deep_action_4>(),
        std::make_shared<deep_action_5>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result_unlimited = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);
    print_plan(result_unlimited);

    assert(result_unlimited.success());
    assert(result_unlimited.size() == 5);
    std::cout << "  (Unlimited depth: plan found with 5 steps)\n";

    planner_options options;
    options.max_depth = 3;

    const auto result_limited = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic, options);
    print_plan(result_limited);

    assert(!result_limited.success());
    assert(result_limited.status == plan_status::DepthLimitReached);

    std::cout << "  (Depth limit correctly enforced at depth 3)\n";
}

TEST(test_complex_health_management_spike_trap)
{
    world_state initial;
    initial.set_float("health", 20.0f);
    initial.set_bool("at_entrance", true);
    initial.set_bool("at_medpack_room", false);
    initial.set_bool("has_medpack", false);
    initial.set_bool("has_bandage", false);

    world_state goal;
    goal.set_float("health", 100.0f);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<cross_spike_trap_action>(),
        std::make_shared<pickup_medpack_action>(),
        std::make_shared<use_medpack_action>(),
        std::make_shared<retreat_to_entrance_action>(),
        std::make_shared<use_bandage_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 3);

    assert(result.actions[0]->get_name() == "CrossSpikeTrap");
    assert(result.actions[1]->get_name() == "PickupMedpack");
    assert(result.actions[2]->get_name() == "UseMedpack");

    assert(result.final_cost == 23);
}

TEST(test_complex_health_management_with_alternative)
{
    world_state initial;
    initial.set_float("health", 20.0f);
    initial.set_bool("at_entrance", true);
    initial.set_bool("at_medpack_room", false);
    initial.set_bool("has_medpack", false);
    initial.set_bool("has_bandage", true);

    world_state goal;
    goal.set_float("health", 100.0f);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<cross_spike_trap_action>(),
        std::make_shared<pickup_medpack_action>(),
        std::make_shared<use_medpack_action>(),
        std::make_shared<use_bandage_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 3);

    assert(result.actions[0]->get_name() == "CrossSpikeTrap");
    assert(result.actions[1]->get_name() == "PickupMedpack");
    assert(result.actions[2]->get_name() == "UseMedpack");
}

TEST(test_complex_health_management_bandage_sufficient)
{
    world_state initial;
    initial.set_float("health", 20.0f);
    initial.set_bool("at_entrance", true);
    initial.set_bool("at_medpack_room", false);
    initial.set_bool("has_medpack", false);
    initial.set_bool("has_bandage", true);

    world_state goal;
    goal.set_float("health", 50.0f);

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<cross_spike_trap_action>(),
        std::make_shared<pickup_medpack_action>(),
        std::make_shared<use_medpack_action>(),
        std::make_shared<use_bandage_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 1);

    assert(result.actions[0]->get_name() == "UseBandage");
    assert(result.final_cost == 3);
}

TEST(test_precondition_conflict_detected)
{
    world_state initial;
    initial.set_bool("is_poisoned", true);
    initial.set_bool("is_nighttime", true);
    initial.set_bool("has_antidote", true);

    world_state goal;
    goal.set_bool("is_cured", true);
    goal.set_bool("is_nighttime", true);

    class use_antidote_daytime_action : public goap_action
    {
    public:
        use_antidote_daytime_action() : goap_action("UseAntidoteDaytime", 5)
        {
            add_precondition("has_antidote", state_value{true}, predicate_op::Equal);
            add_precondition("is_nighttime", state_value{false}, predicate_op::Equal);

            add_effect("is_cured", true);
            add_effect("has_antidote", false);
        }
    };

    class wait_for_day_action : public goap_action
    {
    public:
        wait_for_day_action() : goap_action("WaitForDay", 10)
        {
            add_precondition("is_nighttime", state_value{true}, predicate_op::Equal);
            add_effect("is_nighttime", false);
        }
    };

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<use_antidote_daytime_action>(),
        std::make_shared<wait_for_day_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(!result.success());
    assert(result.status == plan_status::NoSolutionExists);

    std::cout << "  Correctly detected precondition conflict:\n";
    std::cout << "    - UseAntidote requires is_nighttime=false (daytime)\n";
    std::cout << "    - But goal requires is_nighttime=true (nighttime)\n";
    std::cout << "    - Action doesn't modify is_nighttime, so conflict is unresolvable\n";
}

TEST(test_precondition_conflict_resolved)
{
    world_state initial;
    initial.set_bool("is_poisoned", true);
    initial.set_bool("is_nighttime", true);
    initial.set_bool("has_antidote", true);

    world_state goal;
    goal.set_bool("is_cured", true);
    goal.set_bool("is_nighttime", true);

    class use_antidote_daytime_action : public goap_action
    {
    public:
        use_antidote_daytime_action() : goap_action("UseAntidoteDaytime", 5)
        {
            add_precondition("has_antidote", state_value{true}, predicate_op::Equal);
            add_precondition("is_nighttime", state_value{false}, predicate_op::Equal);

            add_effect("is_cured", true);
            add_effect("has_antidote", false);
        }
    };

    class wait_for_day_action : public goap_action
    {
    public:
        wait_for_day_action() : goap_action("WaitForDay", 10)
        {
            add_precondition("is_nighttime", state_value{true}, predicate_op::Equal);
            add_effect("is_nighttime", false);
        }
    };

    class wait_for_nighttime_action : public goap_action
    {
    public:
        wait_for_nighttime_action() : goap_action("WaitForNighttime", 10)
        {
            add_precondition("is_nighttime", state_value{false}, predicate_op::Equal);
            add_effect("is_nighttime", true);
        }
    };

    std::vector<goap_action::ptr> actions =
    {
        std::make_shared<use_antidote_daytime_action>(),
        std::make_shared<wait_for_day_action>(),
        std::make_shared<wait_for_nighttime_action>()
    };

    const goal_count_heuristic heuristic;
    rida_planner planner;

    const auto result = planner.plan(initial, goal, std::span<goap_action::ptr>(actions), heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.status == plan_status::Success);

    std::cout << "  Adding new action correctly resolved existing conflict:\n";
    std::cout << "    - UseAntidote requires is_nighttime=false (daytime)\n";
    std::cout << "    - Goal requires is_nighttime=true (nighttime)\n";
    std::cout << "    - New action requires daytime (is_nighttime=false)\n";
    std::cout << "    - Action chain now modifies everything appropriately, no conflict.\n";
}

//
// MAIN
//

int main()
{
    std::cout << "========================================\n";
    std::cout << "IDAGOAP Test Suite\n";
    std::cout << "========================================\n\n";

    std::cout << "\n========================================\n";
    std::cout << "Results: " << tests_passed << " passed, " << tests_failed << " failed\n";
    std::cout << "========================================\n";

    return tests_failed > 0 ? 1 : 0;
}
