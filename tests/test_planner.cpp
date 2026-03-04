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