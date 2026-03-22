//
// Created by Niall Ó Colmáin on 16/03/2026.
//

#ifndef IDAGOAP_GOAP_AGENT_H
#define IDAGOAP_GOAP_AGENT_H
#include <functional>

#include "async_planner.h"
#include "goal_selector.h"
#include "motive.h"
#include "plan_executor.h"
#include "rida_planner.h"

namespace rida_goap
{
    enum class agent_status
    {
        Idle,
        Planning,
        Executing,
        PlanFailed,
        GoalSatisfied,
        Interrupted
    };

    struct agent_config
    {
        planner_options options{};
        int goal_recheck_interval = 5;
        bool replan_on_world_change = true;
        bool skip_replan_same_goal = true;
        int max_consecutive_failures = 3;
        bool synchronous_planning = false;
    };

    class goap_agent
    {
    public:
        //cb = callback
        using status_changed_cb = std::function<void(agent_status prev, agent_status next)>;
        using goal_selected_cb = std::function<void(const motive&)>;
        using plan_found_cb = std::function<void(const plan_result&)>;
        using action_started_cb = std::function<void(const goap_action&)>;
        using action_finished_cb = std::function<void(const goap_action&, bool success)>;
        using goal_satisfied_cb = std::function<void(const motive&)>;

    private:
        world_state world_model;
        goal_selector selector;
        async_planner planner;
        plan_executor executor;

        agent_config config;
        std::shared_ptr<heuristic> active_heuristic;

        std::vector<goap_action::ptr> actions;

        agent_status status = agent_status::Idle;
        std::shared_ptr<motive> active_motive;
        world_state active_goal_state;

        int tick_counter = 0;
        int consecutive_failures = 0;
        bool world_dirty = false;

        std::shared_ptr<const goap_action> cached_current_action;

        status_changed_cb on_status_changed;
        goal_selected_cb on_goal_selected;
        plan_found_cb on_plan_found;
        action_started_cb on_action_started;
        action_finished_cb on_action_finished;
        goal_satisfied_cb on_goal_satisfied;

    public:
        explicit goap_agent(agent_config config = {});

        goap_agent(const goap_agent&) = delete;
        goap_agent& operator=(const goap_agent&) = delete;
        goap_agent(goap_agent&&) = delete;
        goap_agent& operator=(goap_agent&&) = delete;

        ~goap_agent() = default;

        void tick(float delta_time);

        world_state& get_world_state();
        const world_state& get_world_state() const noexcept;

        void set_world_values(std::initializer_list<std::pair<std::string, state_value>> values);

        void add_action(goap_action::ptr action);
        void remove_action(const std::string& action_name);
        void clear_actions();
        [[nodiscard]] const std::vector<goap_action::ptr>& get_actions() const noexcept;

        void add_motive(std::shared_ptr<motive> new_motive);
        void remove_motive(const std::shared_ptr<motive>& old_motive);
        void clear_motives();

        void set_utility_evaluator(goal_selector::utility_evaluator fn);
        void set_heuristic(std::shared_ptr<heuristic> new_heuristic);

        void interrupt();
        void resume();
        void force_replan();

        [[nodiscard]] agent_status get_status() const noexcept;
        [[nodiscard]] std::shared_ptr<const motive> get_active_motive() const noexcept;
        [[nodiscard]] std::shared_ptr<const goap_action> get_current_action() const noexcept;
        [[nodiscard]] const plan_result& get_current_plan() const noexcept;
        [[nodiscard]] bool is_planning() const noexcept;

        void set_on_status_changed (status_changed_cb cb);
        void set_on_goal_selected (goal_selected_cb cb);
        void set_on_plan_found (plan_found_cb cb);
        void set_on_action_started (action_started_cb cb);
        void set_on_action_finished (action_finished_cb cb);
        void set_on_goal_satisfied (goal_satisfied_cb cb);

    private:
        void transition_to(agent_status next);

        void phase_idle (float delta_time);
        void phase_planning ();
        void phase_executing (float delta_time);

        bool try_select_goal ();
        void kick_off_planning();

        void on_plan_received(const plan_result &result);
        void handle_execution_result(const execution_result& execution_result);

        [[nodiscard]] bool should_recheck_goal() const noexcept;
        [[nodiscard]] bool goal_has_changed(const motive& candidate) const noexcept;
        void mark_world_dirty();

        bool check_and_handle_goal_satisfied(const std::shared_ptr<motive>& motive);
        [[nodiscard]] bool consume_world_dirty() noexcept;
    };
}

#endif //IDAGOAP_GOAP_AGENT_H