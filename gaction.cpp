//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include <algorithm>
#include <ranges>
#include "gworld_model.h"
#include "gaction.h"

bool gaction::check_preconditions(const gworld_model &world_model) const
{
    return std::ranges::all_of(preconditions,[&world_model](const auto& entry)
    {
        const auto& [key, condition] = entry;
        return condition.evaluate(world_model, key);
    });
}

void gaction::apply_effects(gworld_model& world_model) const
{
    for (const auto& [key, value] : effects)
    {
        world_model.set_state(key, value);
    }
}