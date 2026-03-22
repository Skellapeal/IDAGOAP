//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "rida_planner.h"
#include "goap_types.h"

#include <algorithm>
#include <ranges>
#include <unordered_set>
#include <utility>

namespace rida_goap
{
    search_context::search_context(transposition_table& table_ref,
                                   planner_options  opts)
        : options(std::move(opts)),
          table(table_ref),
          start_time(std::chrono::steady_clock::now())
    {
        table.set_max_size(options.max_transposition_size);
    }

    bool search_context::is_time_or_cancelled() noexcept
    {
        const bool has_budget = options.time_budget_ms >= 0;
        const bool can_cancel = options.cancel_token.stop_possible();

        if (!has_budget && !can_cancel) return false;

        const auto now = std::chrono::steady_clock::now();
        const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time);

        const bool timed_out = has_budget &&
                               elapsed > static_cast<std::chrono::milliseconds>(options.time_budget_ms);
        const bool cancelled = can_cancel &&
                               options.cancel_token.stop_requested();

        if (timed_out || cancelled)
        {
            failure_reason = plan_status::TimedOut;
            return true;
        }
        return false;
    }

    bool search_context::should_abort(const int depth) noexcept
    {
        if (is_time_or_cancelled())
        {
            return true;
        }

        if (depth > options.max_depth)
        {
            failure_reason = plan_status::DepthLimitReached;
            return true;
        }

        if (nodes_expanded >= options.max_nodes)
        {
            failure_reason = plan_status::NodeLimitReached;
            return true;
        }

        return false;
    }

    void search_context::record_expansion() noexcept
    {
        ++nodes_expanded;
    }

    void search_context::set_failure(const plan_status status) noexcept
    {
        if (failure_reason == plan_status::Success)
        {
            failure_reason = status;
        }
    }

    bool search_context::has_failure() const noexcept
    {
        return failure_reason != plan_status::Success;
    }

    plan_status search_context::failure() const noexcept
    {
        return failure_reason;
    }

    bool rida_planner::is_action_relevant(
        const goap_action::const_ptr& action,
        const world_state& current_goal)
    {
        const auto& effects = action->get_effects();

        return std::ranges::any_of(
            current_goal.get_states(),
            [&effects](const auto& goal_entry)
            {
                const auto& [key, goal_value] = goal_entry;
                const auto it = effects.find(key);
                return it != effects.end() && it->second == goal_value;
            });
    }

    bool rida_planner::has_precondition_conflict(
        const goap_action::const_ptr& action,
        const world_state& current_goal)
    {
        const auto& effects = action->get_effects();

        return std::ranges::any_of(
            action->get_preconditions(),
            [&current_goal, &effects](const auto& entry)
            {
                const auto& [key, condition] = entry;

                if (effects.find(key) != effects.end()) return false;

                const auto existing_value = current_goal.get_state(key);
                if (!existing_value) return false;

                if (condition.predicate == predicate_op::Equal)
                {
                    return *existing_value != condition.s_value;
                }

                world_state temp;
                temp.set_state(key, *existing_value);
                return !condition.evaluate(temp, key);
            });
    }

    bool rida_planner::is_goal_reached(
        const world_state& regressed_goal,
        const world_state& start)
    {
        return start.satisfies(regressed_goal);
    }

    class goal_regression_guard
    {
        world_state& goal;
        std::vector<std::pair<std::string, std::optional<state_value>>> undo_log{};
        std::unordered_set<std::string> logged_keys{};

    public:
        explicit goal_regression_guard(world_state& g) : goal(g) {}

        bool apply(const goap_action::const_ptr& action,
                   const world_state& initial_state)
        {
            for (const auto& key : action->get_effects() | std::views::keys)
            {
                if (!logged_keys.contains(key))
                {
                    undo_log.emplace_back(key, goal.get_state(key));
                    logged_keys.insert(key);
                }
                goal.remove_state(key);
            }

            for (const auto& [key, condition] : action->get_preconditions())
            {
                if (condition.predicate == predicate_op::Equal)
                {
                    if (!logged_keys.contains(key))
                    {
                        undo_log.emplace_back(key, goal.get_state(key));
                        logged_keys.insert(key);
                    }
                    goal.set_state(key, condition.s_value);
                }
                else
                {
                    const world_state& check_against =
                        goal.has_state(key) ? goal : initial_state;

                    if (!condition.evaluate(check_against, key))
                    {
                        rollback();
                        return false;
                    }
                }
            }

            return true;
        }

        void rollback()
        {
            for (auto& [key, maybe_value] :
                 std::ranges::reverse_view(undo_log))
            {
                if (maybe_value)
                {
                    goal.set_state(key, *maybe_value);
                }
                else
                {
                    goal.remove_state(key);
                }
            }
            undo_log.clear();
            logged_keys.clear();
        }
    };

    bool rida_planner::regressive_ida_search(
        search_context& search_params,
        world_state& current_goal,
        const world_state& initial_state,
        const std::span<const goap_action::const_ptr> sorted_actions,
        const heuristic& heuristic,
        const int accumulated_cost,
        const int cost_limit,
        int& next_cost_limit,
        std::vector<goap_action::const_ptr>& plan,
        const int depth)
    {
        if (search_params.should_abort(depth))
        {
            return false;
        }

        search_params.record_expansion();

        const int predicted_cost = heuristic.estimate(initial_state, current_goal);
        const int total_cost = predicted_cost + accumulated_cost;

        if (search_params.options.use_transposition_table)
        {
            if (const auto cached = search_params.table.lookup(current_goal))
            {
                if (*cached <= total_cost) return false;
            }
        }

        if (total_cost > cost_limit)
        {
            next_cost_limit = std::min(next_cost_limit, total_cost);
            return false;
        }

        if (is_goal_reached(current_goal, initial_state)) return true;

        for (const auto& action : sorted_actions)
        {
            if (!is_action_relevant(action, current_goal)) continue;
            if (has_precondition_conflict(action, current_goal)) continue;

            goal_regression_guard guard(current_goal);
            if (!guard.apply(action, initial_state))
            {
                continue;
            }

            plan.push_back(action);

            if (regressive_ida_search(
                    search_params,
                    current_goal,
                    initial_state,
                    sorted_actions,
                    heuristic,
                    accumulated_cost + action->get_cost(),
                    cost_limit,
                    next_cost_limit,
                    plan,
                    depth + 1))
            {
                return true;
            }

            plan.pop_back();
            guard.rollback();
        }

        if (search_params.options.use_transposition_table)
        {
            search_params.table.store(current_goal, total_cost);
        }

        return false;
    }

    plan_result rida_planner::plan(
        const world_state& initial_state,
        const world_state& goal_state,
        const std::span<goap_action::ptr> available_actions,
        const heuristic& heuristic,
        const planner_options& options)
    {
        plan_result result{};
        current_options = options;

        search_context search_params{transpos_table, current_options};

        std::vector<goap_action::const_ptr> usable_actions;
        usable_actions.reserve(available_actions.size());
        for (const auto& action : available_actions)
        {
            if (action && action->can_run())
            {
                usable_actions.push_back(action);
            }
        }

        if (is_goal_reached(goal_state, initial_state))
        {
            result.status = plan_status::Success;
            result.nodes_expanded = 0;
            result.planning_time_ms = 0;
            return result;
        }

        if (usable_actions.empty())
        {
            result.status = plan_status::NoSolutionExists;
            return result;
        }

        std::ranges::sort(
            usable_actions,
            [](const auto& a, const auto& b){ return a->get_cost() < b->get_cost(); });

        world_state current_goal = goal_state;
        int bound = heuristic.estimate(initial_state, goal_state);
        std::vector<goap_action::const_ptr> plan;

        while (true)
        {
            if (current_options.use_transposition_table) transpos_table.clear();
            int next_bound = std::numeric_limits<int>::max();

            if (regressive_ida_search(
                    search_params,
                    current_goal,
                    initial_state,
                    std::span<const goap_action::const_ptr>(usable_actions.begin(), usable_actions.end()),
                    heuristic,
                    0,
                    bound,
                    next_bound,
                    plan,
                    0))
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

            if (search_params.has_failure())
            {
                result.status = search_params.failure();
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

        result.nodes_expanded = search_params.nodes_expanded;

        const auto end = std::chrono::steady_clock::now();
        result.planning_time_ms =
            static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                                 end - search_params.start_time)
                                 .count());
        return result;
    }
}