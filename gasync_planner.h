//
// Created by Niall Ó Colmáin on 08/03/2026.
//

#ifndef IDAGOAP_GASYNC_PLANNER_H
#define IDAGOAP_GASYNC_PLANNER_H

#include <future>
#include <atomic>
#include <memory>
#include "idaplanner.h"
#include "gplan_result.h"

class gasync_planner
{
public:
    using completion_callback = std::function<void(gplan_result)>;

private:
    idaplanner planner;
    std::future<gplan_result> planning_future;
    std::atomic<bool> is_planning{false};
    std::atomic<bool> should_cancel{false};
    completion_callback on_complete;

public:
    void plan_async(
        const gworld_model &initial_state,
        const gworld_model &goal_state,
        std::vector<gaction::ptr> available_actions,
        std::shared_ptr<gheuristic> heuristic,
        const planner_options &options = planner_options{});

    void plan_async(
        const gworld_model &initial_state,
        const gworld_model &goal_state,
        std::vector<gaction::ptr> available_actions,
        std::shared_ptr<gheuristic> heuristic,
        completion_callback callback,
        const planner_options &options = planner_options{});

    [[nodiscard]] bool is_planning_active() const { return is_planning.load(); }
    void cancel_planning();

    [[nodiscard]] bool try_get_result(gplan_result& out_result);
    gplan_result wait_for_result();

    void set_completion_callback(completion_callback callback)
    {
        on_complete = std::move(callback);
    }
};

#endif //IDAGOAP_GASYNC_PLANNER_H