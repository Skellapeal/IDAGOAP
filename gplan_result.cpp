//
// Created by Niall Ó Colmáin on 04/03/2026.
//

#include "gplan_result.h"

std::string gplan_result::status_string() const
{
    switch (status)
    {
        case gplan_status::Success:
            return "Success";
        case gplan_status::NoSolutionExists:
            return "No solution exists";
        case gplan_status::TimedOut:
            return "Planning Timed out";
        case gplan_status::DepthLimitReached:
            return "Depth limit reached";
        case gplan_status::NodeLimitReached:
            return "Node limit reached";
        default:
            return "Unknown status";
    }
}