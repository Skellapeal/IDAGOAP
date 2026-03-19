//
// Created by Niall Ó Colmáin on 07/03/2026.
//

#include "plan_executor.h"

#include <iostream>

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

    execution_result plan_executor::make_failure(
        const execution_status failure_status,
        std::string reason)
    {
        status = failure_status;
        execution_result result{};
        result.status = status;
        result.current_action_index = current_action_index;
        result.failure_reason = std::move(reason);
        return result;
    }

    execution_result plan_executor::handle_finished_plan()
    {
        std::cout << "Executor: Finished plan\n";
        if (current_action && action_started) end_current_action(true);

        status = execution_status::Success;
        execution_result result{};
        result.status = status;
        result.current_action_index = current_action_index;
        return result;
    }

    void plan_executor::ensure_current_action_loaded()
    {
        if (!current_action && !action_started &&
            current_action_index < current_plan.actions.size())
        {
            current_action = std::const_pointer_cast<goap_action>(current_plan.actions[current_action_index]);
        }
    }

    execution_result plan_executor::handle_pre_start_phase()
    {
        execution_result result{};
        result.status = status;
        result.current_action_index = current_action_index;

        if (action_started) return result;

        if (!validate_preconditions())
        {
            const std::string failure = "Preconditions not met for action: " + current_action->get_name();

            status = execution_status::NeedReplanning;
            result.status = status;
            result.failure_reason = failure;

            if (auto_replan && attempt_replan())
            {
                result.status = execution_status::Running;
                result.failure_reason.clear();
            }
            else return make_failure(execution_status::Failed, failure);
            return result;
        }

        if (!start_current_action())
        {
            const std::string failure = "Failed to start action: " + current_action->get_name();
            return make_failure(execution_status::Failed, failure);
        }
        action_started = true;
        return result;
    }

    execution_result plan_executor::handle_action_tick(const float delta_time)
    {
        execution_result result{};
        result.status = status;
        result.current_action_index = current_action_index;

        const action_status tick_status = current_action->on_tick(delta_time);

        if (tick_status == action_status::Running) return result;
        if (tick_status == action_status::Failed)
        {
            const std::string action_name = current_action->get_name();

            end_current_action(false);
            current_action = nullptr;
            action_started = false;

            status = execution_status::NeedReplanning;
            result.status = status;
            result.failure_reason = "Action failed: " + action_name;

            if (auto_replan && attempt_replan())
            {
                result.status = execution_status::Running;
                result.failure_reason.clear();
                return result;
            }

            return make_failure(execution_status::Failed, "Action failed: " + action_name);
        }

        end_current_action(true);
        current_action = nullptr;
        action_started = false;
        ++current_action_index;

        if (current_action_index >= current_plan.actions.size())
        {
            status = execution_status::Success;
        }

        result.status = status;
        result.current_action_index = current_action_index;
        return result;
    }

    execution_result plan_executor::tick(const float delta_time)
    {
        std::cout << "Executor: Tick\n";
        execution_result result{};
        result.status = status;
        result.current_action_index = current_action_index;

        if (!world_model)
        {
            std::cout << "No world model found\n";
            return make_failure( execution_status::Failed, "No world model set");
        }

        if (!is_running()) return result;
        if (current_action_index >= current_plan.actions.size()) return handle_finished_plan();

        ensure_current_action_loaded();

        result = handle_pre_start_phase();
        if (result.status != execution_status::Running) return result;

        return handle_action_tick(delta_time);
    }

    void plan_executor::interrupt()
    {
        std::cout << "Interrupt";
        if (current_action && action_started) current_action->on_interrupt();

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

        std::cout << "Executor: End current action\n";

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

        if (plan_result new_plan = on_replan_requested(*world_model, goal_state);
            new_plan.success())
        {
            set_plan(std::move(new_plan), saved_goal);
            status = execution_status::Running;
            return true;
        }
        return false;
    }
}
