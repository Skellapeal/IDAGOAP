//
// Created by Niall Ó Colmáin on 07/03/2026.
//

#include "plan_executor.h"

namespace rida_goap
{
    void plan_executor::set_plan(plan_result plan, world_state goal)
    {
        if (is_running()) interrupt();

        current_plan = std::move(plan);
        goal_state = std::move(goal);

        current_action_index = 0;
        current_action = nullptr;

        action_started = false;
        status = current_plan.success() ? execution_status::Running : execution_status::Failed;
    }

    execution_result plan_executor::tick(const float delta_time)
    {
        execution_result result;
        result.status = status;
        result.current_action_index = current_action_index;

        if (!world_model)
        {
            status = execution_status::Failed;
            result.status = status;
            result.failure_reason = "No world model set";
            return result;
        }
        
        if (!is_running()) return result;

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
            current_action = std::const_pointer_cast<goap_action>(
                current_plan.actions[current_action_index]);
            action_started = false;
        }

        if (!action_started)
        {
            if (!validate_preconditions())
            {
                result.failure_reason = "Preconditions not met for action: "
                                        + current_action->get_name();

                status = execution_status::NeedReplanning;
                result.status = status;

                if (auto_replan && attempt_replan())
                {
                    result.status = execution_status::Running;
                }
                else
                {
                    status = execution_status::Failed;
                    result.status = status;
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

        const action_status tick_status = current_action->on_tick(delta_time);

        if (tick_status == action_status::Running)
        {
            result.status = status;
            result.current_action_index = current_action_index;
            return result;
        }

        if (tick_status == action_status::Failed)
        {
            const std::string failed_name = current_plan.actions[current_action_index]->get_name();
            end_current_action(false);
            current_action = nullptr;
            action_started = false;

            result.failure_reason = "Action failed: " + current_plan.actions[current_action_index]->get_name();
            status = execution_status::NeedReplanning;
            result.status = status;

            if (auto_replan && attempt_replan())
            {
                result.status = execution_status::Running;
                return result;
            }

            status = execution_status::Failed;
            result.status = status;
            return result;
        }

        end_current_action(true);
        current_action = nullptr;
        action_started = false;
        ++current_action_index;

        if (current_action_index >= current_plan.actions.size())
        {
            status = execution_status::Success;
        }

        result.current_action_index = current_action_index;
        result.status = status;
        return result;
    }

    void plan_executor::interrupt()
    {
        if (current_action && action_started)
        {
            current_action->on_interrupt();
        }

        status = execution_status::Interrupted;
        current_action = nullptr;
        action_started = false;
    }

    void plan_executor::reset()
    {
        interrupt();
        current_plan = plan_result{};
        goal_state = world_state{};
        current_action_index = 0;
        status = execution_status::Idle;
    }

    bool plan_executor::start_current_action() const
    {
        return current_action && current_action->on_start();
    }

    void plan_executor::end_current_action(const bool success) const
    {
        if (!current_action) return;

        if (success && world_model)
        {
            if (current_action->check_preconditions(*world_model))
            {
                current_action->apply_effects(*world_model);
            }
        }
        current_action->on_end(success);
    }

    bool plan_executor::validate_preconditions() const
    {
        if (!current_action || !world_model) return false;
        return current_action->check_preconditions(*world_model);
    }

    bool plan_executor::attempt_replan()
    {
        if (!on_replan_requested || !world_model) return false;

        const world_state saved_goal = goal_state;

        if (plan_result new_plan = on_replan_requested(*world_model, goal_state); new_plan.success())
        {
            set_plan(std::move(new_plan), saved_goal);
            status = execution_status::Running;
            return true;
        }
        return false;
    }
}