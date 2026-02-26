//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "idaplanner.h"
#include <algorithm>
#include <ranges>
#include <vector>
#include <limits>

bool idaplanner::inverse_depth_first_search(
    gworld_model& current_goal,
    const gworld_model& initial_state,
    const std::span<const gaction*> available_actions,
    const gheuristic& heuristic,
    const int accumulated_cost, const int cost_limit, int& next_cost_limit,
    std::vector<const gaction*>& plan)
{
    const int predicted_cost = heuristic.estimate(initial_state, current_goal);
    const int total_cost = predicted_cost + accumulated_cost;

    if (auto cached_cost = table.lookup(current_goal))
    {
        if (*cached_cost <= total_cost)
        {
            return false;
        }
    }

    if (total_cost > cost_limit)
    {
        next_cost_limit = std::min(next_cost_limit, total_cost);
        return false;
    }

    if (is_goal_reached(current_goal, initial_state))
    {
        return true;
    }

    for (const gaction* action : available_actions)
    {
        bool relevant = false;
        for (const auto& [key, goal_value] : current_goal.get_states())
        {
            if (const auto& effects = action->get_effects(); effects.contains(key)
                && effects.at(key) == goal_value)
            {
                relevant = true;
                break;
            }
        }

        if (!relevant) continue;

        const gworld_model previous_goal = current_goal;

        for (const auto &key: action->get_effects() | std::views::keys)
        {
            current_goal.remove_state(key);
        }
        for (const auto& [key, value] : action->get_preconditions())
        {
            current_goal.set_state(key, value);
        }

        plan.push_back(action);

        if (inverse_depth_first_search(current_goal, initial_state, available_actions, heuristic,
                                       accumulated_cost + action->get_cost(), cost_limit, next_cost_limit, plan))
        {
            return true;
        }

        plan.pop_back();
        current_goal = previous_goal;
    }

    table.store(current_goal, total_cost);
    return false;
}

bool idaplanner::is_goal_reached(const gworld_model& regressed_goal, const gworld_model& start) {
    for (const auto& [key, value] : regressed_goal.get_states())
    {
        if (auto start_value = start.get_state(key); !start_value || *start_value != value)
        {
            return false;
        }
    }
    return true;
}

std::vector<const gaction*> idaplanner::plan(
    const gworld_model& initial_state,
    const gworld_model& goal_state,
    const std::span<const gaction*> available_actions,
    const gheuristic& heuristic)
{
    gworld_model current_goal = goal_state;
    int bound = heuristic.estimate(initial_state, goal_state);
    std::vector<const gaction*> plan;

    while (true)
    {
        table.clear();
        int next_bound = std::numeric_limits<int>::max();

        if (inverse_depth_first_search(current_goal, initial_state, available_actions, heuristic,
                                       0,bound, next_bound, plan))
        {
            std::ranges::reverse(plan);
            return plan;
        }

        if (next_bound == std::numeric_limits<int>::max())
        {
            return {};
        }

        bound = next_bound;
        current_goal = goal_state;
        plan.clear();
    }
}