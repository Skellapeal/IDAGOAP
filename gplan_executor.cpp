//
// Created by ismis on 07/03/2026.
//

#include "gplan_executor.h"

void gplan_executor::set_plan(gplan_result plan)
{
    if (is_running())
    {
        interrupt();
    }

    current_plan = std::move(plan);
    current_action_index = 0;
    current_action = nullptr;
    action_started = false;
    status = current_plan.success() ? execution_status::Running : execution_status::Failed;
}

execution_result gplan_executor::tick(const float delta_time)
{
    execution_result result;
    result.status = status;
    result.current_action_index = current_action_index;

    if (!is_running()) return result;
    if (!world_model)
    {
        status = execution_status::Failed;
        result.status = status;
        result.failure_reason = "No world model set";
        return result;
    }

    if (current_action_index >= current_plan.actions.size())
    {
        if (current_action && action_started)
        {
            end_current_action(true);
        }
        status = execution_status::Success;
        result.status = status;
        return result;
    }

    if (!current_action)
    {
        current_action = std::const_pointer_cast<gaction>(current_plan.actions[current_action_index]);
        action_started = false;
    }

    if (!action_started)
    {
        if (!validate_preconditions())
        {
            status = execution_status::NeedReplanning;
            result.status = status;
            result.failure_reason = "Preconditions not met for action: " + current_action->get_name();

            if (auto_replan && attempt_replan())
            {
                result.status = execution_status::Running;
                return tick(delta_time);
            }
            return result;
        }

        if (!start_current_action())
        {
            status = execution_status::Failed;
            result.status = status;
            result.failure_reason = "Failed to start action: " + current_action->get_name();
            return result;
        }
        action_started = true;
    }

    current_action->on_tick(delta_time);

    // temporary
    end_current_action(true);
    current_action = nullptr;
    action_started = false;
    current_action_index++;

    result.current_action_index = current_action_index;
    result.status = status;
    return result;
}

void gplan_executor::interrupt()
{
    if (current_action && action_started)
    {
        current_action->on_interrupt();
        end_current_action(false);
    }

    status = execution_status::Interrupted;
    current_action = nullptr;
    action_started = false;
}

void gplan_executor::reset()
{
    interrupt();
    current_plan = gplan_result{};
    current_action_index = 0;
    status = execution_status::Success;
}

bool gplan_executor::start_current_action() const
{
    if (!current_action) return false;

    const bool started = current_action->on_start();
    return started;
}

void gplan_executor::end_current_action(const bool success) const
{
    if (!current_action) return;

    if (success && world_model)
    {
        current_action->apply_effects(*world_model);
    }

    current_action->on_end(success);
}

bool gplan_executor::validate_preconditions() const
{
    if (!current_action || !world_model) return false;
    return current_action->check_preconditions(*world_model);
}

bool gplan_executor::attempt_replan()
{
    if (!on_replan_needed || !world_model) return false;

    if (gplan_result new_plan = on_replan_needed(*world_model); new_plan.success())
    {
        set_plan(std::move(new_plan));
        status = execution_status::Running;
        return true;
    }
    return false;
}