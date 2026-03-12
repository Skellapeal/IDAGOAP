//
// Created by Niall Ó Colmáin on 07/03/2026.
//

#ifndef IDAGOAP_GPLAN_EXECUTOR_H
#define IDAGOAP_GPLAN_EXECUTOR_H

#include <memory>
#include <functional>
#include "gplan_result.h"
#include "gworld_model.h"

enum class execution_status
{
    Running,
    Success,
    Failed,
    Interrupted,
    NeedReplanning
};

struct execution_result
{
    execution_status status = execution_status::Running;
    size_t current_action_index = 0;
    std::string failure_reason;
};

class gplan_executor
{
public:
    using replan_callback = std::function<gplan_result(const gworld_model&)>;

private:
    gplan_result current_plan;
    size_t current_action_index = 0;
    std::shared_ptr<gaction> current_action;
    execution_status status = execution_status::Running;

    replan_callback on_replan_needed;
    gworld_model* world_model = nullptr;

    bool action_started = false;
    bool auto_replan = true;

public:
    explicit gplan_executor(gworld_model* world = nullptr) : world_model(world) {}

    void set_plan(gplan_result plan);
    void set_world_model(gworld_model* world) { world_model = world;}
    void set_replan_callback(replan_callback callback) { on_replan_needed = std::move(callback); }
    void set_auto_replan(const bool enable) { auto_replan = enable; }

    [[nodiscard]] execution_status get_status() const { return status; }
    [[nodiscard]] bool is_running() const { return status == execution_status::Running; }
    [[nodiscard]] bool is_complete() const { return status == execution_status::Success || status == execution_status::Failed; }
    [[nodiscard]] size_t get_current_action_index() const { return current_action_index; }
    [[nodiscard]] std::shared_ptr<const gaction> get_current_action() const { return current_action; }
    [[nodiscard]] const gplan_result& get_plan() const { return current_plan; }

    execution_result tick(float delta_time);
    void interrupt();
    void reset();

private:
    [[nodiscard]] bool start_current_action() const;
    void end_current_action(bool success) const;
    [[nodiscard]] bool validate_preconditions() const;
    bool attempt_replan();
};


#endif //IDAGOAP_GPLAN_EXECUTOR_H