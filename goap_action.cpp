//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include <algorithm>
#include <ranges>
#include "world_state.h"
#include "goap_action.h"

namespace rida_goap
{
    bool goap_action::check_preconditions(const world_state &world_model) const
    {
        return std::ranges::all_of(preconditions,[&world_model](const auto& entry)
        {
            const auto& [key, condition] = entry;
            return condition.evaluate(world_model, key);
        });
    }

    void goap_action::apply_effects(world_state& world_model) const
    {
        for (const auto& [key, value] : effects)
        {
            world_model.set_state(key, value);
        }
    }
}