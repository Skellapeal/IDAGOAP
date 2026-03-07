//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GACTION_H
#define IDAGOAP_GACTION_H

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include "gtypes.h"

class gworld_model;

class gaction : public std::enable_shared_from_this<gaction>
{
protected:
    std::string name;
    int cost = 0.0f;

    std::unordered_map<std::string, gcondition> preconditions;
    std::unordered_map<std::string, gvalue> effects;

public:
    using ptr = std::shared_ptr<gaction>;
    using const_ptr = std::shared_ptr<const gaction>;

    gaction(std::string name, const int cost) : name(std::move(name)), cost(cost) {}

    virtual ~gaction() = default;

    [[nodiscard]] const std::string& get_name() const { return name; }
    [[nodiscard]] int get_cost() const { return cost; }
    [[nodiscard]] const std::unordered_map<std::string, gcondition>& get_preconditions() const { return preconditions; }
    [[nodiscard]] const std::unordered_map<std::string, gvalue>& get_effects() const { return effects; }

    void add_precondition(const std::string& key, const gvalue& value, const gcomparison comparison = gcomparison::Equal)
    {
        preconditions[key] = gcondition(key, value, comparison);
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

    bool check_preconditions(const gworld_model& world_model) const;
    void apply_effects(gworld_model& world_model) const;

    virtual bool on_start() { return true; }
    virtual void on_tick(float delta_time) {}
    virtual void on_end(bool success) {}
    virtual void on_interrupt() {}
};

#endif //IDAGOAP_GACTION_H