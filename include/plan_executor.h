//
// Created by Niall Ó Colmáin on 07/03/2026.
//

#ifndef IDAGOAP_PLAN_EXECUTOR_H
#define IDAGOAP_PLAN_EXECUTOR_H

#include <memory>
#include <functional>

#include "plan_result.h"
#include "world_state.h"

namespace rida_goap
{
    enum class execution_status
    {
        Idle,
        Running,
        Success,
        Failed,
        Interrupted,
        NeedReplanning
    };

    struct execution_result
    {
        execution_status status = execution_status::Idle;
        size_t current_action_index = 0;
        std::string failure_reason;
    };

    class plan_executor
    {
    public:
        using replan_callback = std::function<plan_result(const world_state& current_world, const world_state& goal)>;

    private:
        plan_result current_plan{};
        world_state goal_state{};

        size_t current_action_index = 0;
        goap_action::ptr current_action{};
        execution_status status = execution_status::Idle;

        replan_callback on_replan_requested{};
        world_state* world_model = nullptr;

        bool action_started = false;
        bool just_started = false;
        bool auto_replan = true;

    public:
        explicit plan_executor(world_state* world_model = nullptr) : world_model(world_model) {}

        void set_plan(plan_result plan, world_state goal);
        void set_world_model(world_state* world) { world_model = world; }
        void set_replan_callback(replan_callback callback)
        {
            on_replan_requested = std::move(callback);
        }
        void set_auto_replan(bool enable) { auto_replan = enable; }

        [[nodiscard]] execution_status get_status() const noexcept { return status; }
        [[nodiscard]] bool is_running() const noexcept { return status == execution_status::Running; }
        [[nodiscard]] bool is_complete() const noexcept
        {
            return status == execution_status::Success ||
                   status == execution_status::Failed;
        }
        [[nodiscard]] size_t get_current_action_index() const noexcept
        {
            return current_action_index;
        }
        [[nodiscard]] std::shared_ptr<const goap_action> get_current_action() const noexcept
        {
            return current_action;
        }
        [[nodiscard]] const plan_result& get_plan() const noexcept
        {
            return current_plan;
        }

        execution_result tick(float delta_time);
        void interrupt();
        void reset();

    private:
        [[nodiscard]] bool start_current_action() const;
        void end_current_action(bool success) const;
        [[nodiscard]] bool validate_preconditions() const;
        bool attempt_replan();

        [[nodiscard]] execution_result make_failure(
            execution_status failure_status,
            std::string      reason);

        [[nodiscard]] execution_result handle_finished_plan();
        void ensure_current_action_loaded();
        [[nodiscard]] execution_result handle_pre_start_phase();
        [[nodiscard]] execution_result handle_action_tick(float delta_time);
    };
}

#endif // IDAGOAP_PLAN_EXECUTOR_H