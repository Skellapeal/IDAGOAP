//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_IDAPLANNER_H
#define IDAGOAP_IDAPLANNER_H
#include <span>

#include "gaction.h"
#include "gheuristic.h"
#include "gtranpos_table.h"

class idaplanner
{
    gtranpos_table table;

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
        const gheuristic &heuristic);
};

#endif //IDAGOAP_IDAPLANNER_H