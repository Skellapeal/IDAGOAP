//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GACTION_H
#define IDAGOAP_GACTION_H

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include "goap_types.h"

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

    std::unordered_map<std::string, gcondition> preconditions;
    std::unordered_map<std::string, gvalue> effects;

public:
    using ptr = std::shared_ptr<goap_action>;
    using const_ptr = std::shared_ptr<const goap_action>;

    goap_action(std::string name, const int cost) : name(std::move(name)), cost(cost) {}

    virtual ~goap_action() = default;

    [[nodiscard]] const std::string& get_name() const { return name; }
    [[nodiscard]] int get_cost() const { return cost; }
    [[nodiscard]] const std::unordered_map<std::string, gcondition>& get_preconditions() const { return preconditions; }
    [[nodiscard]] const std::unordered_map<std::string, gvalue>& get_effects() const { return effects; }

    void add_precondition(const std::string& key, const gvalue& value, const gcomparison comparison = gcomparison::Equal)
    {
        preconditions[key] = gcondition(value, comparison);
    }

    void add_effect(const std::string& key, const gvalue& value)
    {
        effects[key] = value;
    }

    /**
    * @brief Checks if this action is statically available.
     *
     * ONLY check for properties that CANNOT change during plan execution:
     * - Agent skills/capabilities
     * - Global game flags
     * - Permission/authorization
     *
     * DO NOT check for:
     * - Resources (might be acquired mid-plan)
     * - Items (might be crafted mid-plan)
     * - Dynamic world state
     *
     * Use preconditions for dynamic requirements instead.
     */
    [[nodiscard]] virtual bool can_run() const { return true; }

    bool check_preconditions(const world_state& world_model) const;
    void apply_effects(world_state& world_model) const;

    virtual bool on_start() { return true; }
    virtual action_status on_tick(float delta_time) { return action_status::Running; }
    virtual void on_end(bool success) {}
    virtual void on_interrupt() {}
};

#endif //IDAGOAP_GACTION_H