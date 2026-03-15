//
// Created by Niall Ó Colmáin on 08/03/2026.
//

#include "async_planner.h"

namespace rida_goap
{
    void async_planner::plan_async(
        const world_state& initial_state,
        const world_state& goal_state,
        std::vector<goap_action::ptr> available_actions,
        std::shared_ptr<heuristic> heuristic,
        const planner_options& options)
    {
        if (is_planning.load()) cancel_planning();

        stop_source = std::stop_source{};
        is_planning.store(true);

        planner_options effective_options = options;
        effective_options.cancel_token = stop_source.get_token();

        const auto completion_handler = on_completed;

        planning_future = std::async(std::launch::async,
            [this,
                initial_state, goal_state,
                actions = std::move(available_actions),
                heuristic = std::move(heuristic),
                effective_options,
                completion_handler]() mutable -> plan_result
            {
                auto result = planner.plan(initial_state, goal_state, actions, *heuristic, effective_options);
                is_planning.store(false);

                if (completion_handler && !effective_options.cancel_token.stop_requested())
                {
                    completion_handler(result);
                }
                return result;
            }
        );
    }

    void async_planner::plan_async(
        const world_state& initial_state,
        const world_state& goal_state,
        std::vector<goap_action::ptr> available_actions,
        std::shared_ptr<heuristic> heuristic,
        completion_callback callback,
        const planner_options& options)
    {
        set_completion_callback(std::move(callback));
        plan_async(initial_state, goal_state, std::move(available_actions),
                   std::move(heuristic), options);
    }

    void async_planner::cancel_planning()
    {
        if (stop_source.request_stop())
        {
            if (planning_future.valid())
            {
                planning_future.wait();
            }

            is_planning.store(false);
        }
    }

    bool async_planner::try_get_result(plan_result& out_result)
    {
        if (!planning_future.valid()) return false;

        if (planning_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            out_result = planning_future.get();
            return true;
        }

        return false;
    }

    plan_result async_planner::wait_for_result()
    {
        if (planning_future.valid())
        {
            return planning_future.get();
        }

        return plan_result{};
    }
}