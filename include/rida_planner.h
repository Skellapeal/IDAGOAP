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
#include <stop_token>

#include "goap_action.h"
#include "heuristic.h"
#include "transposition_table.h"
#include "plan_result.h"

namespace rida_goap
{
    struct planner_options
    {
        int     max_depth              = std::numeric_limits<int>::max();
        int64_t max_nodes              = std::numeric_limits<int64_t>::max();
        int     time_budget_ms         = -1;
        bool    use_transposition_table= true;
        size_t  max_transposition_size = std::numeric_limits<size_t>::max();
        std::stop_token cancel_token{};
    };

    struct search_context
    {
        planner_options                     options{};
        transposition_table&                table;
        int64_t                             nodes_expanded   = 0;
        plan_status                         failure_reason   = plan_status::Success;
        std::chrono::steady_clock::time_point start_time{};

        explicit search_context(transposition_table& table_ref,
                                planner_options  opts);

        [[nodiscard]] bool should_abort(int depth) noexcept;
        void record_expansion() noexcept;
        void set_failure(plan_status status) noexcept;
        [[nodiscard]] bool has_failure() const noexcept;
        [[nodiscard]] plan_status failure() const noexcept;

        [[nodiscard]] bool is_time_or_cancelled() noexcept;
    };

    class rida_planner
    {
        transposition_table transpos_table;

        planner_options current_options{};

        [[nodiscard]] static bool is_goal_reached(
            const world_state& regressed_goal,
            const world_state& start);

        [[nodiscard]] static bool is_action_relevant(
            const goap_action::const_ptr& action,
            const world_state& current_goal);

        [[nodiscard]] static bool has_precondition_conflict(
            const goap_action::const_ptr& action,
            const world_state& current_goal);

        bool regressive_ida_search(
            search_context& search_params,
            world_state& current_goal,
            const world_state& initial_state,
            std::span<const goap_action::const_ptr> sorted_actions,
            const heuristic& heuristic,
            int accumulated_cost,
            int cost_limit,
            int& next_cost_limit,
            std::vector<goap_action::const_ptr>& plan,
            int depth);

    public:
        [[nodiscard]] plan_result plan(
            const world_state& initial_state,
            const world_state& goal_state,
            std::span<goap_action::ptr> available_actions,
            const heuristic& heuristic,
            const planner_options& options);

        [[nodiscard]] plan_result plan(
            const world_state& initial_state,
            const world_state& goal_state,
            std::span<goap_action::ptr> available_actions,
            const heuristic& heuristic)
        {
            const planner_options default_options{};
            return plan(initial_state, goal_state, available_actions, heuristic, default_options);
        }
    };
}

#endif // IDAGOAP_RIDA_PLANNER_H