//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "rida_planner.h"
#include "goap_types.h"
#include <algorithm>
#include <atomic>
#include <ranges>
#include <vector>
#include <limits>

namespace rida_goap
{
    bool rida_planner::is_action_relevant(const goap_action::const_ptr& action, const world_state &current_goal)
    {
        const auto& effects = action->get_effects();

        const bool relevant = std::ranges::any_of(current_goal.get_states(),
            [&effects](const std::pair<const std::string, state_value>& goal_entry)
        {
            const auto& [key, goal_value] = goal_entry;
            const bool contains = effects.contains(key);
            const bool matches = contains && effects.at(key) == goal_value;

            return matches;
        });

        return relevant;
    }


    bool rida_planner::has_precondition_conflict(const goap_action::const_ptr& action, const world_state &current_goal)
    {
        const auto& effects = action->get_effects();

        return std::ranges::any_of(action->get_preconditions(),
            [&current_goal, &effects](const std::pair<const std::string, state_condition>& precondition_entry)
            {
                const auto& [precondition_key, precondition_condition] = precondition_entry;

                if (effects.contains(precondition_key)) return false;
                if (precondition_condition.predicate != predicate_op::Equal) return false;

                if (const auto existing_value = current_goal.get_state(precondition_key))
                {
                    return *existing_value != precondition_condition.s_value;
                }
                return false;
            }
        );
    }

    bool rida_planner::is_goal_reached(const world_state& regressed_goal, const world_state& start)
    {
        return start.satisfies(regressed_goal);
    }

    bool rida_planner::regressive_ida_search(
        world_state& current_goal,
        const world_state& initial_state,
        const std::span<goap_action::const_ptr> available_actions,
        const heuristic &heuristic,
        const int accumulated_cost, const int cost_limit, int& next_cost_limit,
        std::vector<goap_action::const_ptr>& plan,
        const int depth)
    {
        if (current_options.time_budget_ms >= 0 || current_options.cancel_token != nullptr)
        {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);
            const bool timed_out = current_options.time_budget_ms >= 0
                && elapsed > static_cast<std::chrono::milliseconds>(current_options.time_budget_ms);
            const bool cancelled = current_options.cancel_token != nullptr
                && current_options.cancel_token->load(std::memory_order_relaxed);

            if (timed_out || cancelled)
            {
                failure_reason = plan_status::TimedOut;
                return false;
            }
        }

        if (depth > current_options.max_depth)
        {
            failure_reason = plan_status::DepthLimitReached;
            return false;
        }

        ++nodes_expanded;
        if (nodes_expanded > current_options.max_nodes)
        {
            failure_reason = plan_status::NodeLimitReached;
            return false;
        }

        const int predicted_cost = heuristic.estimate(initial_state, current_goal);
        const int total_cost = predicted_cost + accumulated_cost;

        if (current_options.use_transposition_table)
        {
            if (const auto cached_cost = transpos_table.lookup(current_goal))
            {
                if (*cached_cost <= total_cost)
                {
                    return false;
                }
            }
        }

        if (total_cost > cost_limit)
        {
            next_cost_limit = std::min(next_cost_limit, total_cost);
            return false;
        }

        if (is_goal_reached(current_goal, initial_state)) return true;

        for (const auto& action : available_actions)
        {
            if (!is_action_relevant(action, current_goal)) continue;
            if (has_precondition_conflict(action, current_goal)) continue;

            const world_state previous_goal = current_goal;

            // Remove effects from the goal to allow for regression
            for (const auto &key: action->get_effects() | std::views::keys)
            {
                current_goal.remove_state(key);
            }

            for (const auto& [key, condition] : action->get_preconditions())
            {
                if (condition.predicate == predicate_op::Equal)
                {
                    current_goal.set_state(key, condition.s_value);
                }
            }

            plan.push_back(action);

            if (regressive_ida_search(current_goal, initial_state, available_actions, heuristic,
                                           accumulated_cost + action->get_cost(), cost_limit, next_cost_limit, plan, depth + 1))
            {
                return true;
            }

            plan.pop_back();
            current_goal = previous_goal;
        }

        if (current_options.use_transposition_table)
        {
            transpos_table.store(current_goal, total_cost);
        }

        return false;
    }

    plan_result rida_planner::plan(
        const world_state &initial_state,
        const world_state &goal_state,
        const std::span<goap_action::ptr> available_actions,
        const heuristic &heuristic,
        const planner_options &options)
    {
        plan_result result;
        current_options = options;
        nodes_expanded = 0;
        failure_reason = plan_status::Success;
        start_time = std::chrono::steady_clock::now();

        transpos_table.set_max_size(current_options.max_transposition_size);

        std::vector<goap_action::const_ptr> usable_actions;
        usable_actions.reserve(available_actions.size());
        for (const auto& action : available_actions)
        {
            if (action -> can_run())
            {
                usable_actions.push_back(action);
            }
        }

        if (usable_actions.empty())
        {
            if (is_goal_reached(goal_state, initial_state))
            {
                result.status = plan_status::Success;
                result.nodes_expanded = 0;
            }
            else
            {
                result.status = plan_status::NoSolutionExists;
            }
            return result;
        }

        if (is_goal_reached(goal_state, initial_state))
        {
            result.status = plan_status::Success;
            result.nodes_expanded = 0;
            result.planning_time_ms = 0;
            return result;
        }

        world_state current_goal = goal_state;
        int bound = heuristic.estimate(initial_state, goal_state);
        std::vector<goap_action::const_ptr> plan;

        while (true)
        {
            transpos_table.clear();
            int next_bound = std::numeric_limits<int>::max();

            if (regressive_ida_search(current_goal, initial_state, usable_actions, heuristic,
                                           0,bound, next_bound, plan))
            {
                std::ranges::reverse(plan);
                result.actions = std::move(plan);
                result.status = plan_status::Success;
                result.final_cost = 0;

                for (const auto& action : result.actions)
                {
                    result.final_cost += action->get_cost();
                }
                break;
            }

            if (failure_reason != plan_status::Success)
            {
                result.status = failure_reason;
                break;
            }

            if (next_bound == std::numeric_limits<int>::max())
            {
                result.status = plan_status::NoSolutionExists;
                break;
            }

            bound = next_bound;
            current_goal = goal_state;
            plan.clear();
        }

        result.nodes_expanded = nodes_expanded;

        const auto planning_end = std::chrono::steady_clock::now();
        result.planning_time_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(planning_end - start_time).count());

        return result;
    }
}