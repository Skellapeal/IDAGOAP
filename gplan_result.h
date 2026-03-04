//
// Created by ismis on 04/03/2026.
//

#ifndef IDAGOAP_GPLAN_RESULT_H
#define IDAGOAP_GPLAN_RESULT_H
#include <string>
#include <vector>


class gaction;

enum class gplan_status
{
    Success,                    // Found and build a plan
    NoSolutionExists,           // No action sequence satisfies the goal
    TimedOut,                   // Time budget elapsed
    DepthLimitReached,          // Hit max depth
    NodeLimitReached            // Hit max nodes
};

struct gplan_result
{
    std::vector<const gaction*> actions;
    gplan_status status = gplan_status::Success;

    // Debug
    int nodes_expanded = 0;
    int final_cost = 0;
    int planning_time_ms = 0;

    [[nodiscard]] bool success() const { return status == gplan_status::Success; }
    [[nodiscard]] bool empty() const { return actions.empty(); }
    [[nodiscard]] size_t size() const { return actions.size(); }

    [[nodiscard]] std::string status_string() const;
};

#endif //IDAGOAP_GPLAN_RESULT_H