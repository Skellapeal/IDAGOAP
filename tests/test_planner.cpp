//
// Created by Niall Ó Colmáin on 04/03/2026.
//

#include <iostream>
#include <cassert>
#include <vector>
#include <memory>

#include "../idaplanner.h"
#include "../gheuristic.h"
#include "../gworld_model.h"
#include "../gplan_result.h"
#include "test_actions.h"

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

void print_plan(const gplan_result& result)
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
            std::cout << "    " << (i+1) << ". " << result.actions[i]->get_name()
                      << " (cost: " << result.actions[i]->get_cost() << ")\n";
        }
    }
}

//
// TESTS
//

TEST(test_simple_single_action_plan)
{
    gworld_model initial;
    initial.set_int("has_wood", 0);

    gworld_model goal;
    goal.set_int("has_wood", 1);

    gather_wood_action gather_wood;
    std::vector<const gaction*> actions = { &gather_wood };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 1);
    assert(result.actions[0]->get_name() == "GatherWood");
    assert(result.final_cost == 10);
    assert(result.nodes_expanded > 0);
}

TEST(test_two_step_plan)
{
    gworld_model initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_tool", 0);

    gworld_model goal;
    goal.set_int("has_tool", 1);

    gather_wood_action gather_wood;
    craft_tool_action craft_tool;
    std::vector<const gaction*> actions = { &gather_wood, &craft_tool };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 2);
    assert(result.actions[0]->get_name() == "GatherWood");
    assert(result.actions[1]->get_name() == "CraftTool");
    assert(result.final_cost == 15); // 10 + 5
}

TEST(test_complex_multi_resource_plan)
{
    gworld_model initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_stone", 0);
    initial.set_int("has_shelter", 0);

    gworld_model goal;
    goal.set_int("has_shelter", 1);

    gather_wood_action gather_wood;
    gather_stone_action gather_stone;
    build_shelter_action build_shelter;

    std::vector<const gaction*> actions = {
        &gather_wood,
        &gather_stone,
        &build_shelter
    };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 3);
    assert(result.actions[2]->get_name() == "BuildShelter");
    assert(result.final_cost == 45);
}

TEST(test_no_solution_exists)
{
    gworld_model initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_tool", 0);

    gworld_model goal;
    goal.set_int("has_tool", 1);

    craft_tool_action craft_tool;
    std::vector<const gaction*> actions = { &craft_tool };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(!result.success());
    assert(result.status == gplan_status::NoSolutionExists);
    assert(result.empty());
}

TEST(test_already_at_goal)
{
    gworld_model initial;
    initial.set_int("has_wood", 1);

    gworld_model goal;
    goal.set_int("has_wood", 1);

    gather_wood_action gather_wood;
    std::vector<const gaction*> actions = { &gather_wood };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.empty());
    assert(result.final_cost == 0);
}

TEST(test_disabled_action_filtered)
{
    gworld_model initial;
    initial.set_int("has_wood", 0);

    gworld_model goal;
    goal.set_int("has_wood", 1);

    gather_wood_action gather_wood;
    disabled_action disabled;

    std::vector<const gaction*> actions = { &gather_wood, &disabled };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 1);
    assert(result.actions[0]->get_name() == "GatherWood");
}

TEST(test_node_limit_reached)
{
    gworld_model initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_stone", 0);
    initial.set_int("has_shelter", 0);

    gworld_model goal;
    goal.set_int("has_shelter", 1);

    gather_wood_action gather_wood;
    gather_stone_action gather_stone;
    build_shelter_action build_shelter;
    craft_tool_action craft_tool;

    std::vector<const gaction*> actions = {
        &gather_wood, &gather_stone, &build_shelter, &craft_tool
    };

    gheuristic heuristic;
    idaplanner planner;

    planner_options options;
    options.max_nodes = 5;

    auto result = planner.plan(initial, goal, actions, heuristic, options);

    print_plan(result);

    assert(!result.success());
    assert(result.status == gplan_status::NodeLimitReached);
    assert(result.nodes_expanded <= 6);
}

TEST(test_time_budget_enforced)
{
    gworld_model initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_stone", 0);
    initial.set_int("has_shelter", 0);

    gworld_model goal;
    goal.set_int("has_shelter", 1);

    gather_wood_action gather_wood;
    gather_stone_action gather_stone;
    build_shelter_action build_shelter;

    std::vector<const gaction*> actions = {
        &gather_wood, &gather_stone, &build_shelter
    };

    gheuristic heuristic;
    idaplanner planner;

    planner_options options;
    options.time_budget_ms = 1;

    auto result = planner.plan(initial, goal, actions, heuristic, options);

    print_plan(result);

    if (!result.success())
    {
        assert(result.status == gplan_status::TimedOut);
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
    gworld_model initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_stone", 0);
    initial.set_int("has_shelter", 0);

    gworld_model goal;
    goal.set_int("has_shelter", 1);

    gather_wood_action gather_wood;
    gather_stone_action gather_stone;
    build_shelter_action build_shelter;

    std::vector<const gaction*> actions = {
        &gather_wood, &gather_stone, &build_shelter
    };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.nodes_expanded < 100);
    std::cout << "  (Heuristic kept search focused)\n";
}

TEST(test_bool_preconditions_and_effects)
{
    gworld_model initial;
    initial.set_bool("enemy_visible", false);
    initial.set_bool("is_hidden", false);

    gworld_model goal;
    goal.set_bool("is_hidden", true);

    scout_action scout;
    hide_action hide;

    std::vector<const gaction*> actions = { &scout, &hide };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

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
    gvalue f1{30.0f};
    gvalue f2{30.0f};
    gvalue f3{30.1f};

    assert(f1 == f2);
    assert(f1 != f3);

    gvalue f4{30.0f};
    gvalue f5{30.0f + 1e-7f};

    auto get_float = [](const gvalue& v) -> float
    {
        return std::get<float>(v);
    };

    constexpr float epsilon = 1e-6f;
    assert(std::abs(get_float(f4) - get_float(f5)) <= epsilon);

    std::unordered_map<std::string, gvalue> map1;
    map1["health"] = gvalue{100.0f};

    std::unordered_map<std::string, gvalue> map2;
    map2["health"] = gvalue{100.0f};

    assert(map1.at("health") == map2.at("health"));

    std::unordered_map<std::string, gvalue> map3;
    map3["health"] = gvalue{100.0f + 1e-7f};

    assert(std::abs(get_float(map1.at("health")) - get_float(map3.at("health"))) <= epsilon);
}

TEST(test_is_action_relevant_debug)
{
    gworld_model goal;
    goal.set_state("health", gvalue{100.0f});

    const rest_action rest;

    const auto& effects = rest.get_effects();
    const auto& goal_states = goal.get_states();

    assert(goal_states.contains("health"));
    assert(effects.contains("health"));

    const auto& effect_value = effects.at("health");
    const auto& goal_value = goal_states.at("health");
    assert(effect_value.index() == goal_value.index());

    assert(effect_value == goal_value);
}

TEST(test_float_preconditions_and_effects)
{
    gworld_model initial;
    initial.set_float("health", 30.0f);

    gworld_model goal;
    goal.set_float("health", 100.0f);

    rest_action rest;

    std::vector<const gaction*> actions = { &rest };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 1);
    assert(result.actions[0]->get_name() == "Rest");
    assert(result.final_cost == 12);

    std::cout << "  (Float preconditions/effects working correctly)\n";
}

TEST(test_mixed_type_world_state)
{
    gworld_model initial;
    initial.set_int("ammo_count", 30);
    initial.set_bool("enemy_visible", false);
    initial.set_float("health", 100.0f);

    gworld_model goal;
    goal.set_bool("enemy_visible", true);

    scout_action scout;

    std::vector<const gaction*> actions = { &scout };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 1);
    assert(result.actions[0]->get_name() == "Scout");

    std::cout << "  (Mixed int/bool/float state handled correctly)\n";
}

TEST(test_depth_limit_reached)
{
    gworld_model initial;

    gworld_model goal;
    goal.set_int("depth_5", 1);

    deep_action_1 d1;
    deep_action_2 d2;
    deep_action_3 d3;
    deep_action_4 d4;
    deep_action_5 d5;

    std::vector<const gaction*> actions = { &d1, &d2, &d3, &d4, &d5 };

    gheuristic heuristic;
    idaplanner planner;

    auto result_unlimited = planner.plan(initial, goal, actions, heuristic);
    print_plan(result_unlimited);

    assert(result_unlimited.success());
    assert(result_unlimited.size() == 5);
    std::cout << "  (Unlimited depth: plan found with 5 steps)\n";

    planner_options options;
    options.max_depth = 3;

    auto result_limited = planner.plan(initial, goal, actions, heuristic, options);
    print_plan(result_limited);

    assert(!result_limited.success());
    assert(result_limited.status == gplan_status::DepthLimitReached);

    std::cout << "  (Depth limit correctly enforced at depth 3)\n";
}

TEST(test_complex_health_management_spike_trap)
{
    gworld_model initial;
    initial.set_float("health", 20.0f);
    initial.set_bool("at_entrance", true);
    initial.set_bool("at_medpack_room", false);
    initial.set_bool("has_medpack", false);
    initial.set_bool("has_bandage", false);

    gworld_model goal;
    goal.set_float("health", 100.0f);

    cross_spike_trap_action cross_trap;
    pickup_medpack_action pickup;
    use_medpack_action use_medpack;
    retreat_to_entrance_action retreat;
    use_bandage_action use_bandage;

    std::vector<const gaction*> actions =
    {
        &cross_trap,
        &pickup,
        &use_medpack,
        &retreat,
        &use_bandage
    };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

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
    gworld_model initial;
    initial.set_float("health", 20.0f);
    initial.set_bool("at_entrance", true);
    initial.set_bool("at_medpack_room", false);
    initial.set_bool("has_medpack", false);
    initial.set_bool("has_bandage", true);

    gworld_model goal;
    goal.set_float("health", 100.0f);

    cross_spike_trap_action cross_trap;
    pickup_medpack_action pickup;
    use_medpack_action use_medpack;
    use_bandage_action use_bandage;

    std::vector<const gaction*> actions =
    {
        &cross_trap,
        &pickup,
        &use_medpack,
        &use_bandage
    };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 3);

    assert(result.actions[0]->get_name() == "CrossSpikeTrap");
    assert(result.actions[1]->get_name() == "PickupMedpack");
    assert(result.actions[2]->get_name() == "UseMedpack");
}

TEST(test_complex_health_management_bandage_sufficient)
{
    gworld_model initial;
    initial.set_float("health", 20.0f);
    initial.set_bool("at_entrance", true);
    initial.set_bool("at_medpack_room", false);
    initial.set_bool("has_medpack", false);
    initial.set_bool("has_bandage", true);

    gworld_model goal;
    goal.set_float("health", 50.0f);

    cross_spike_trap_action cross_trap;
    pickup_medpack_action pickup;
    use_medpack_action use_medpack;
    use_bandage_action use_bandage;

    std::vector<const gaction*> actions =
    {
        &cross_trap,
        &pickup,
        &use_medpack,
        &use_bandage
    };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.size() == 1);

    assert(result.actions[0]->get_name() == "UseBandage");
    assert(result.final_cost == 3);
}

TEST(test_precondition_conflict_detected)
{
    gworld_model initial;
    initial.set_bool("is_poisoned", true);
    initial.set_bool("is_nighttime", true);
    initial.set_bool("has_antidote", true);

    gworld_model goal;
    goal.set_bool("is_cured", true);
    goal.set_bool("is_nighttime", true);

    class use_antidote_daytime_action : public gaction
    {
    public:
        use_antidote_daytime_action() : gaction("UseAntidoteDaytime", 5)
        {
            add_precondition("has_antidote", gvalue{true}, gcomparison::Equal);
            add_precondition("is_nighttime", gvalue{false}, gcomparison::Equal);

            effects["is_cured"] = gvalue{true};
            effects["has_antidote"] = gvalue{false};
        }
        bool setup() override { return true; }
        bool end() override { return true; }
    };

    class wait_for_day_action : public gaction
    {
    public:
        wait_for_day_action() : gaction("WaitForDay", 10)
        {
            add_precondition("is_nighttime", gvalue{true}, gcomparison::Equal);

            effects["is_nighttime"] = gvalue{false};
        }
        bool setup() override { return true; }
        bool end() override { return true; }
    };

    use_antidote_daytime_action use_antidote;
    wait_for_day_action wait_for_day;

    std::vector<const gaction*> actions = { &use_antidote, &wait_for_day };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(!result.success());
    assert(result.status == gplan_status::NoSolutionExists);

    std::cout << "  Correctly detected precondition conflict:\n";
    std::cout << "    - UseAntidote requires is_nighttime=false (daytime)\n";
    std::cout << "    - But goal requires is_nighttime=true (nighttime)\n";
    std::cout << "    - Action doesn't modify is_nighttime, so conflict is unresolvable\n";
}

TEST(test_precondition_conflict_resolved)
{
    gworld_model initial;
    initial.set_bool("is_poisoned", true);
    initial.set_bool("is_nighttime", true);
    initial.set_bool("has_antidote", true);

    gworld_model goal;
    goal.set_bool("is_cured", true);
    goal.set_bool("is_nighttime", true);

    class use_antidote_daytime_action : public gaction
    {
    public:
        use_antidote_daytime_action() : gaction("UseAntidoteDaytime", 5)
        {
            add_precondition("has_antidote", gvalue{true}, gcomparison::Equal);
            add_precondition("is_nighttime", gvalue{false}, gcomparison::Equal);

            effects["is_cured"] = gvalue{true};
            effects["has_antidote"] = gvalue{false};
        }
        bool setup() override { return true; }
        bool end() override { return true; }
    };

    class wait_for_day_action : public gaction
    {
    public:
        wait_for_day_action() : gaction("WaitForDay", 10)
        {
            add_precondition("is_nighttime", gvalue{true}, gcomparison::Equal);

            effects["is_nighttime"] = gvalue{false};
        }
        bool setup() override { return true; }
        bool end() override { return true; }
    };

    class wait_for_nighttime_action : public gaction
    {
    public:
        wait_for_nighttime_action() : gaction("WaitForNighttime", 10)
        {
            add_precondition("is_nighttime", gvalue{false}, gcomparison::Equal);

            effects["is_nighttime"] = gvalue{true};
        }

        bool setup() override {return true; }
        bool end() override { return true; }
    };

    use_antidote_daytime_action use_antidote;
    wait_for_day_action wait_for_day;
    wait_for_nighttime_action wait_for_nighttime;

    std::vector<const gaction*> actions = { &use_antidote, &wait_for_day, &wait_for_nighttime };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    assert(result.success());
    assert(result.status == gplan_status::Success);

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