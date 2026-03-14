//
// Created by Niall Ó Colmáin on 08/03/2026.
//

#include "async_planner.h"

void async_planner::plan_async(
    const world_state& initial_state,
    const world_state& goal_state,
    std::vector<goap_action::ptr> available_actions,
    std::shared_ptr<heurisitc> heuristic,
    const planner_options& options)
{
    if (is_planning.load()) cancel_planning();

    is_planning.store(true);
    should_cancel.store(false);

    const auto completion_handler = on_completed;

    planning_future = std::async(std::launch::async,
        [this,
            initial_state, goal_state,
            actions = std::move(available_actions),
            heuristic = std::move(heuristic),
            options,
            completion_handler]() mutable -> plan_result
        {
            auto result = planner.plan(initial_state, goal_state, actions, *heuristic, options);
            is_planning.store(false);

            if (completion_handler && !should_cancel.load()) completion_handler(result);
            return result;
        }
    );
}

void async_planner::plan_async(
    const world_state& initial_state,
    const world_state& goal_state,
    std::vector<goap_action::ptr> available_actions,
    std::shared_ptr<heurisitc> heuristic,
    completion_callback callback,
    const planner_options& options)
{
    set_completion_callback(std::move(callback));
    plan_async(initial_state, goal_state, std::move(available_actions),
               std::move(heuristic), options);
}

void async_planner::cancel_planning()
{
    should_cancel.store(true);

    if (planning_future.valid())
    {
        planning_future.wait();
    }

    is_planning.store(false);
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