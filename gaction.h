//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GACTION_H
#define IDAGOAP_GACTION_H

#include <string>
#include <unordered_map>
#include <utility>

class gworld_model;

class gaction
{
protected:
    std::string name;
    int cost = 0.0f;

    std::unordered_map<std::string, int> preconditions;
    std::unordered_map<std::string, int> effects;

public:
    gaction(std::string name, const int cost) : name(std::move(name)), cost(cost) {}

    virtual ~gaction() = default;

    [[nodiscard]] const std::string& get_name() const { return name; }
    [[nodiscard]] int get_cost() const { return cost; }
    [[nodiscard]] const std::unordered_map<std::string, int>& get_preconditions() const { return preconditions; }
    [[nodiscard]] const std::unordered_map<std::string, int>& get_effects() const { return effects; }

    bool check_preconditions(const gworld_model* world_model) const;
    void apply_effects(gworld_model* world_model) const;

    virtual bool setup() = 0;
    virtual bool end() = 0;
};

#endif //IDAGOAP_GACTION_H