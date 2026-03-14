//
// Created by Niall Ó Colmáin on 08/03/2026.
//

#ifndef IDAGOAP_ASYNC_PLANNER_H
#define IDAGOAP_ASYNC_PLANNER_H

#include <future>
#include <atomic>
#include <memory>
#include "rida_planner.h"
#include "plan_result.h"

namespace rida_goap
{
    class async_planner
    {
    public:
        using completion_callback = std::function<void(plan_result)>;

    private:
        rida_planner planner;
        std::future<plan_result> planning_future;
        std::atomic<bool> is_planning{false};
        std::atomic<bool> should_cancel{false};
        completion_callback on_completed;

    public:
        void plan_async(
            const world_state &initial_state,
            const world_state &goal_state,
            std::vector<goap_action::ptr> available_actions,
            std::shared_ptr<heuristic> heuristic,
            const planner_options &options = planner_options{});

        void plan_async(
            const world_state &initial_state,
            const world_state &goal_state,
            std::vector<goap_action::ptr> available_actions,
            std::shared_ptr<heuristic> heuristic,
            completion_callback callback,
            const planner_options &options = planner_options{});

        [[nodiscard]] bool is_planning_active() const { return is_planning.load(); }
        void cancel_planning();

        [[nodiscard]] bool try_get_result(plan_result& result);
        plan_result wait_for_result();

        void set_completion_callback(completion_callback callback)
        {
            on_completed = std::move(callback);
        }
    };
}

#endif //IDAGOAP_ASYNC_PLANNER_H