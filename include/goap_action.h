//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GOAP_ACTION_H
#define IDAGOAP_GOAP_ACTION_H

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include "goap_types.h"

namespace rida_goap
{
    class world_state;

    enum class action_status
    {
        Running,
        Succeeded,
        Failed
    };

    class goap_action
    {
    protected:
        std::string name;
        int cost = 0;

        std::unordered_map<std::string, state_condition> preconditions;
        std::unordered_map<std::string, state_value> effects;

    public:
        using ptr = std::shared_ptr<goap_action>;
        using const_ptr = std::shared_ptr<const goap_action>;

        goap_action(std::string name, const int cost) : name(std::move(name)), cost(cost) {}
        virtual ~goap_action() = default;

        goap_action(const goap_action&) = default;
        goap_action(goap_action&&) noexcept = default;
        goap_action& operator=(const goap_action&) = default;
        goap_action& operator=(goap_action&&) noexcept = default;

        [[nodiscard]] const std::string& get_name() const { return name; }
        [[nodiscard]] int get_cost() const { return cost; }
        [[nodiscard]] const std::unordered_map<std::string, state_condition>& get_preconditions() const { return preconditions; }
        [[nodiscard]] const std::unordered_map<std::string, state_value>& get_effects() const { return effects; }

        void add_precondition(const std::string& key,
                              state_value value,
                              const predicate_op comparison = predicate_op::Equal)
        {
            preconditions[key] = state_condition(std::move(value), comparison);
        }

        void add_effect(const std::string& key, const state_value& value){ effects[key] = value; }

        [[nodiscard]] virtual bool can_run() const { return true; }
        bool check_preconditions(const world_state& world_model) const;
        void apply_effects(world_state& world_model) const;

        virtual bool on_start() { return true; }
        virtual action_status on_tick(float delta_time) = 0;
        virtual void on_end(bool) {}
        virtual void on_interrupt() {}
    };

    class instant_goap_action : public goap_action
    {
    public:
        instant_goap_action(std::string name, const int cost) : goap_action(std::move(name), cost) {}

        action_status on_tick(float) final
        {
            return execute() ? action_status::Succeeded : action_status::Failed;
        }

    protected:
        virtual bool execute() { return true; }
    };
}

#endif // IDAGOAP_GOAP_ACTION_H