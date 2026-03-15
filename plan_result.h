//
// Created by Niall Ó Colmáin on 04/03/2026.
//

#ifndef IDAGOAP_PLAN_RESULT_H
#define IDAGOAP_PLAN_RESULT_H
#include <string>
#include <vector>
#include <memory>
#include "goap_action.h"

namespace rida_goap
{
    enum class plan_status
    {
        Success,                    // Found and built a plan
        NoSolutionExists,           // No action sequence satisfies the goal
        TimedOut,                   // Time budget elapsed
        DepthLimitReached,          // Hit max depth
        NodeLimitReached,            // Hit max nodes
        Cancelled
    };

    struct plan_result
    {
        std::vector<goap_action::const_ptr> actions;
        plan_status status = plan_status::Success;

        int64_t  nodes_expanded = 0;
        int final_cost = 0;
        int planning_time_ms = 0;

        [[nodiscard]] bool success() const { return status == plan_status::Success; }
        [[nodiscard]] bool is_trivially_satisfied() const { return status == plan_status::Success && actions.empty(); }
        [[nodiscard]] bool has_no_actions() const { return actions.empty(); }
        [[nodiscard]] size_t size() const { return actions.size(); }

        [[nodiscard]] std::string status_string() const;
    };
}

#endif //IDAGOAP_PLAN_RESULT_H