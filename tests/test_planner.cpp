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

// Simple test counter
int tests_passed = 0;
int tests_failed = 0;

// Test macro
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

// Helper: print plan
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
    // Setup: Need wood, and we can gather it
    gworld_model initial;
    initial.set_int("has_wood", 0);

    gworld_model goal;
    goal.set_int("has_wood", 1);

    gather_wood_action gather_wood;
    std::vector<const gaction*> actions = { &gather_wood };

    gheuristic heuristic;
    idaplanner planner;

    // Plan
    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    // Verify
    assert(result.success());
    assert(result.size() == 1);
    assert(result.actions[0]->get_name() == "GatherWood");
    assert(result.final_cost == 10);
    assert(result.nodes_expanded > 0);
}

TEST(test_two_step_plan)
{
    // Setup: Need a tool, requires gathering wood first
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

    // Verify correct sequence: gather wood -> craft tool
    assert(result.success());
    assert(result.size() == 2);
    assert(result.actions[0]->get_name() == "GatherWood");
    assert(result.actions[1]->get_name() == "CraftTool");
    assert(result.final_cost == 15); // 10 + 5
}

TEST(test_complex_multi_resource_plan)
{
    // Setup: Build shelter requires wood + stone
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

    // Verify: should gather both resources then build
    assert(result.success());
    assert(result.size() == 3);
    // Order of gathering doesn't matter, but build must be last
    assert(result.actions[2]->get_name() == "BuildShelter");
    assert(result.final_cost == 45); // 10 + 15 + 20
}

TEST(test_no_solution_exists)
{
    // Setup: Want a tool but no gather wood action available
    gworld_model initial;
    initial.set_int("has_wood", 0);
    initial.set_int("has_tool", 0);

    gworld_model goal;
    goal.set_int("has_tool", 1);

    craft_tool_action craft_tool;
    // Only craft action available, can't get wood!
    std::vector<const gaction*> actions = { &craft_tool };

    gheuristic heuristic;
    idaplanner planner;

    auto result = planner.plan(initial, goal, actions, heuristic);

    print_plan(result);

    // Verify failure
    assert(!result.success());
    assert(result.status == gplan_status::NoSolutionExists);
    assert(result.empty());
}

TEST(test_already_at_goal)
{
    // Setup: Already have what we want
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

    // Should return empty plan with success
    assert(result.success());
    assert(result.empty());
    assert(result.final_cost == 0);
}

TEST(test_disabled_action_filtered)
{
    // Setup: Disabled action should be filtered by can_run()
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

    // Should succeed using only enabled action
    assert(result.success());
    assert(result.size() == 1);
    assert(result.actions[0]->get_name() == "GatherWood");
}

TEST(test_node_limit_reached)
{
    // Setup: Complex problem with very low node limit
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
    options.max_nodes = 5; // Very restrictive

    auto result = planner.plan(initial, goal, actions, heuristic, options);

    print_plan(result);

    // Should hit node limit
    assert(!result.success());
    assert(result.status == gplan_status::NodeLimitReached);
    assert(result.nodes_expanded <= 6); // May expand one more before checking
}

TEST(test_time_budget_enforced)
{
    // Setup: Same complex problem with tiny time budget
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
    options.time_budget_ms = 1; // 1ms - very tight

    auto result = planner.plan(initial, goal, actions, heuristic, options);

    print_plan(result);

    // Might succeed (if fast) or timeout
    if (!result.success())
    {
        assert(result.status == gplan_status::TimedOut);
        std::cout << "  (Correctly timed out)\n";
    }
    else
    {
        std::cout << "  (Succeeded within time budget)\n";
    }

    // Either way, time should be recorded
    assert(result.planning_time_ms >= 0);
}

TEST(test_heuristic_provides_guidance)
{
    // Setup: Same as earlier but track that heuristic reduces search
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

    // With heuristic, should solve efficiently
    assert(result.success());
    assert(result.nodes_expanded < 100); // Should be much less
    std::cout << "  (Heuristic kept search focused)\n";
}

//
// MAIN
//

int main()
{
    std::cout << "========================================\n";
    std::cout << "IDAGOAP Test Suite\n";
    std::cout << "========================================\n\n";

    // Tests auto-run via static initialization

    std::cout << "\n========================================\n";
    std::cout << "Results: " << tests_passed << " passed, " << tests_failed << " failed\n";
    std::cout << "========================================\n";

    return tests_failed > 0 ? 1 : 0;
}
