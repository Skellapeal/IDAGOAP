//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_IDAPLANNER_H
#define IDAGOAP_IDAPLANNER_H
#include <chrono>
#include <limits>
#include <span>
#include <vector>
#include "gaction.h"
#include "gheuristic.h"
#include "gtranpos_table.h"

struct planner_options
{
    int max_depth = std::numeric_limits<int>::max();
    int max_nodes = std::numeric_limits<int>::max();
    int time_budget_ms = -1;
    bool use_transposition_table = true;
};

class idaplanner
{
    gtranpos_table table;
    int nodes_expanded = 0;
    planner_options current_options;
    std::chrono::steady_clock::time_point start_time;

    bool inverse_depth_first_search(
        gworld_model &current_goal,
        const gworld_model &initial_state,
        std::span<const gaction*> available_actions,
        const gheuristic &heuristic,
        int accumulated_cost, int cost_limit, int &next_cost_limit,
        std::vector<const gaction*> &plan);

    [[nodiscard]] static bool is_goal_reached(const gworld_model& regressed_goal, const gworld_model& start);

    [[nodiscard]] static bool is_action_relevant(const gaction* action, const gworld_model& current_goal);
    [[nodiscard]] static bool has_precondition_conflict(const gaction *action, const gworld_model &current_goal);

public:
    std::vector<const gaction*> plan(
        const gworld_model &initial_state,
        const gworld_model &goal_state,
        std::span<const gaction*> available_actions,
        const gheuristic &heuristic,
        const planner_options &options);

    std::vector<const gaction*> plan(
        const gworld_model &initial_state,
        const gworld_model &goal_state,
        std::span<const gaction*> available_actions,
        const gheuristic &heuristic)
    {
        constexpr planner_options default_options;
        return plan(initial_state, goal_state, available_actions, heuristic, default_options);
    }
};

#endif //IDAGOAP_IDAPLANNER_H