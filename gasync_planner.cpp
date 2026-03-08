//
// Created by ismis on 08/03/2026.
//

#include "gasync_planner.h"

void gasync_planner::plan_async(
    const gworld_model& initial_state,
    const gworld_model& goal_state,
    std::vector<gaction::const_ptr> available_actions,
    std::shared_ptr<gheuristic> heuristic,
    const planner_options& options)
{
    if (is_planning.load())
    {
        cancel_planning();
    }

    is_planning.store(true);
    should_cancel.store(false);

    planning_future = std::async(std::launch::async,
        [this, initial_state, goal_state, actions = std::move(available_actions),
         heuristic = std::move(heuristic), options]() mutable -> gplan_result
        {
            auto result = planner.plan(
                initial_state,
                goal_state,
                actions,
                *heuristic,
                options
            );

            is_planning.store(false);

            if (on_complete && !should_cancel.load())
            {
                on_complete(result);
            }

            return result;
        }
    );
}

void gasync_planner::plan_async(
    const gworld_model& initial_state,
    const gworld_model& goal_state,
    std::vector<gaction::const_ptr> available_actions,
    std::shared_ptr<gheuristic> heuristic,
    completion_callback callback,
    const planner_options& options)
{
    set_completion_callback(std::move(callback));
    plan_async(initial_state, goal_state, std::move(available_actions),
               std::move(heuristic), options);
}

void gasync_planner::cancel_planning()
{
    should_cancel.store(true);

    if (planning_future.valid())
    {
        planning_future.wait();
    }

    is_planning.store(false);
}

bool gasync_planner::try_get_result(gplan_result& out_result)
{
    if (!planning_future.valid()) return false;

    if (planning_future.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
    {
        out_result = planning_future.get();
        return true;
    }

    return false;
}

gplan_result gasync_planner::wait_for_result()
{
    if (planning_future.valid())
    {
        return planning_future.get();
    }

    return gplan_result{};
}