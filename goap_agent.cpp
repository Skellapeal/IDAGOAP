//
// Created by Niall Ó Colmáin on 16/03/2026.
//

#include "goap_agent.h"

namespace rida_goap
{
    goap_agent::goap_agent(agent_config config) : executor(&world_model), config(std::move(config))
    {
        executor.set_replan_callback([this, config](const world_state& current, const world_state& goal) -> plan_result
        {
            if (!active_heuristic) return plan_result{};

            rida_planner sync_planner;
            std::vector<goap_action::ptr> action_copy = actions;
            return sync_planner.plan(current, goal,
                                     std::span{action_copy},
                                     *active_heuristic,
                                     config.options);
        });

        executor.set_auto_replan(false);
    }

    void goap_agent::tick(float delta_time)
    {
        ++tick_counter;

        switch (status)
        {
            case agent_status::Idle:
            case agent_status::GoalSatisfied:
            case agent_status::PlanFailed:
                phase_idle(delta_time);
                break;
            case agent_status::Planning:
                phase_planning();
                break;
            case agent_status::Executing:
                phase_executing(delta_time);
                break;
            case agent_status::Interrupted:
                break;
        }
    }

    void goap_agent::phase_idle(float)
    {
        if (actions.empty()) return;
        if (try_select_goal()) kick_off_planning();
    }

    bool goap_agent::try_select_goal()
    {
        const auto candidate = selector.select_goal(world_model);
        if (!candidate) return false;

        if (candidate->is_satisfied(world_model))
        {
            if (on_goal_satisfied) on_goal_satisfied(*candidate);

            transition_to(agent_status::GoalSatisfied);
            return false;
        }

        const bool same_goal = active_motive &&
                               config.skip_replan_same_goal &&
                               !goal_has_changed(*candidate);

        if (same_goal && status == agent_status::Executing) return false;

        active_motive = candidate;
        active_goal_state = candidate->get_goal_state();

        if (on_goal_selected) on_goal_selected(*candidate);
        return true;
    }

    void goap_agent::kick_off_planning()
    {
        if (!active_heuristic) throw std::runtime_error("goap_agent: no heuristic set before planning");

        transition_to(agent_status::Planning);

        planner.plan_async(
            world_model,
            active_goal_state,
            actions,
            active_heuristic,
            [this](plan_result result)
            {
                on_plan_received(std::move(result));
            }, config.options);
    }

    void goap_agent::phase_planning()
    {
        if (!planner.is_planning_active() && status == agent_status::Planning)
        {
            plan_result result;
            if (planner.try_get_result(result))
            {
                on_plan_received(std::move(result));
            }
        }
    }

    void goap_agent::on_plan_received(const plan_result &result)
    {
        if (!result.success() || result.actions.empty())
        {
            ++consecutive_failures;
            if (consecutive_failures >= config.max_consecutive_failures)
            {
                transition_to(agent_status::Idle);
                consecutive_failures = 0;
            }
            else
            {
                transition_to(agent_status::PlanFailed);
            }
            return;
        }

        consecutive_failures = 0;
        if (on_plan_found) on_plan_found(result);

        executor.set_plan(result, active_goal_state);
        transition_to(agent_status::Executing);
    }

    void goap_agent::phase_executing(float delta_time)
    {
        if (active_motive && active_motive->is_satisfied(world_model))
        {
            executor.interrupt();
            if (on_goal_satisfied) on_goal_satisfied(*active_motive);
            transition_to(agent_status::GoalSatisfied);
            return;
        }

        if (config.replan_on_world_change && world_dirty && should_recheck_goal())
        {
            world_dirty = false;
            const auto candidate = selector.select_goal(world_model);
            if (candidate && goal_has_changed(*candidate))
            {
                executor.interrupt();
                planner.cancel_planning();
                active_motive = candidate;
                active_goal_state = candidate->get_goal_state();
                if (on_goal_selected) on_goal_selected(*candidate);
                kick_off_planning();
                return;
            }
        }

        const auto execution_result = executor.tick(delta_time);
        handle_execution_result(execution_result);
    }

    void goap_agent::handle_execution_result(const execution_result &execution_result)
    {
        switch (execution_result.status)
        {
            case execution_status::Running:
                if (const auto act = executor.get_current_action(); act && on_action_started)
                {
                    on_action_started(*act);
                }
                break;
            case execution_status::Success:
                if (on_action_finished && executor.get_current_action())
                {
                    on_action_finished(*executor.get_current_action(), true);
                }
                transition_to(agent_status::Idle);
                break;
            case execution_status::Failed:
            case execution_status::NeedReplanning:
                if (on_action_finished && executor.get_current_action())
                {
                    on_action_finished(*executor.get_current_action(), false);
                }
                force_replan();
                break;
            case execution_status::Interrupted:
                transition_to(agent_status::Idle);
                break;
            default:
                break;
        }
    }

    world_state& goap_agent::get_world_state() noexcept
    {
        mark_world_dirty();
        return world_model;
    }

    const world_state& goap_agent::get_world_state() const noexcept { return world_model; }

    void goap_agent::set_world_values(std::initializer_list<std::pair<std::string, state_value>> values)
    {
        for (auto& [key, val] : values)
        {
            world_model.set_state(key, val);
        }
        mark_world_dirty();
    }

    void goap_agent::mark_world_dirty() { world_dirty = true; }

    void goap_agent::add_action(goap_action::ptr action)
    {
        actions.push_back(std::move(action));
    }

    void goap_agent::remove_action(const std::string& action_name)
    {
        std::erase_if(actions, [&](const goap_action::ptr& a)
                      { return a->get_name() == action_name; });
    }

    void goap_agent::clear_actions() { actions.clear(); }

    const std::vector<goap_action::ptr>& goap_agent::get_actions() const noexcept { return actions; }

    void goap_agent::add_motive(std::shared_ptr<motive> new_motive)
    {
        selector.add_motive(std::move(new_motive));
    }

    void goap_agent::remove_motive(const std::shared_ptr<motive>& old_motive)
    {
        selector.remove_motive(old_motive);
    }

    void goap_agent::clear_motives() { selector.clear_motives(); }

    void goap_agent::set_utility_evaluator(goal_selector::utility_evaluator fn)
    {
        selector.set_utility_evaluator(std::move(fn));
    }

    void goap_agent::set_heuristic(std::shared_ptr<heuristic> new_heuristic)
    {
        active_heuristic = std::move(new_heuristic);
    }

    void goap_agent::interrupt()
    {
        planner.cancel_planning();
        executor.interrupt();
        transition_to(agent_status::Interrupted);
    }

    void goap_agent::resume()
    {
        if (status == agent_status::Interrupted) transition_to(agent_status::Idle);
    }

    void goap_agent::force_replan()
    {
        planner.cancel_planning();
        executor.reset();

        if (try_select_goal()) kick_off_planning();
        else transition_to(agent_status::Idle);
    }

    agent_status goap_agent::get_status() const noexcept { return status; }
    std::shared_ptr<const motive> goap_agent::get_active_motive() const noexcept { return active_motive; }

    std::shared_ptr<const goap_action> goap_agent::get_current_action() const noexcept
    {
        return executor.get_current_action();
    }

    const plan_result& goap_agent::get_current_plan() const noexcept { return executor.get_plan(); }
    bool goap_agent::is_planning() const noexcept { return status == agent_status::Planning; }

    void goap_agent::set_on_status_changed (status_changed_cb cb) { on_status_changed = std::move(cb); }
    void goap_agent::set_on_goal_selected (goal_selected_cb cb) { on_goal_selected = std::move(cb); }
    void goap_agent::set_on_plan_found (plan_found_cb cb) { on_plan_found = std::move(cb); }
    void goap_agent::set_on_action_started (action_started_cb cb) { on_action_started = std::move(cb); }
    void goap_agent::set_on_action_finished (action_finished_cb cb) { on_action_finished = std::move(cb); }
    void goap_agent::set_on_goal_satisfied (goal_satisfied_cb cb) { on_goal_satisfied = std::move(cb); }

    void goap_agent::transition_to(const agent_status next)
    {
        if (status == next) return;

        const agent_status prev = status;
        status = next;
        if (on_status_changed) on_status_changed(prev, next);
    }

    bool goap_agent::should_recheck_goal() const noexcept
    {
        if (config.goal_recheck_interval <= 0) return true;
        return tick_counter % config.goal_recheck_interval == 0;
    }

    bool goap_agent::goal_has_changed(const motive& candidate) const noexcept
    {
        if (!active_motive) return true;
        return candidate.get_goal_state() != active_motive->get_goal_state();
    }
}
