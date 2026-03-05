//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gaction.h"

#include <algorithm>
#include <ranges>
#include "gworld_model.h"

bool gaction::check_preconditions(const gworld_model* world_model) const
{
    if (!world_model) return false;

    return std::ranges::all_of(
        preconditions | std::views::values,
        [world_model](const auto& condition) { return condition.evaluate(*world_model); });
}

void gaction::apply_effects(gworld_model* world_model) const
{
    for (const auto& [key, value] : effects)
    {
        world_model->set_state(key, value);
    }
}