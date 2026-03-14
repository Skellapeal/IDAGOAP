//
// Created by Niall Ó Colmáin on 04/03/2026.
//

#include "plan_result.h"

namespace rida_goap
{
    std::string plan_result::status_string() const
    {
        switch (status)
        {
            case plan_status::Success:
                return "Success";
            case plan_status::NoSolutionExists:
                return "No solution exists";
            case plan_status::TimedOut:
                return "Planning Timed out";
            case plan_status::DepthLimitReached:
                return "Depth limit reached";
            case plan_status::NodeLimitReached:
                return "Node limit reached";
            default:
                return "Unknown status";
        }
    }
}