//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_RIDA_PLANNER_H
#define IDAGOAP_RIDA_PLANNER_H

#include <chrono>
#include <limits>
#include <span>
#include <vector>
#include <memory>
#include "goap_action.h"
#include "heuristic.h"
#include "transposition_table.h"
#include "plan_result.h"

struct planner_options
{
    int max_depth = std::numeric_limits<int>::max();
    int64_t max_nodes = std::numeric_limits<int64_t>::max();
    int time_budget_ms = -1;
    bool use_transposition_table = true;
    size_t max_transposition_size = std::numeric_limits<size_t>::max();
    const std::atomic<bool>* cancel_token = nullptr;
};

class rida_planner
{
    transposition_table transposition_table;
    int64_t nodes_expanded = 0;
    planner_options current_options;
    std::chrono::steady_clock::time_point start_time;
    gplan_status failure_reason = gplan_status::Success;

    bool regressive_ida_search(
        world_state &current_goal,
        const world_state &initial_state,
        std::span<goap_action::const_ptr> available_actions,
        const heuristic &heuristic,
        int accumulated_cost, int cost_limit, int &next_cost_limit,
        std::vector<goap_action::const_ptr> &plan,
        int depth = 0);

    [[nodiscard]] static bool is_goal_reached(const world_state& regressed_goal, const world_state& start);
    [[nodiscard]] static bool is_action_relevant(const goap_action::const_ptr& action, const world_state& current_goal);
    [[nodiscard]] static bool has_precondition_conflict(const goap_action::const_ptr& action, const world_state &current_goal);

public:
    [[nodiscard]] plan_result plan(
        const world_state &initial_state,
        const world_state &goal_state,
        std::span<goap_action::ptr> available_actions,
        const heuristic &heuristic,
        const planner_options &options);

    [[nodiscard]] plan_result plan(
        const world_state &initial_state,
        const world_state &goal_state,
        const std::span<goap_action::ptr> available_actions,
        const heuristic &heuristic)
    {
        constexpr planner_options default_options;
        return plan(initial_state, goal_state, available_actions, heuristic, default_options);
    }
};

#endif //IDAGOAP_RIDA_PLANNER_H