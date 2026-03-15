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
        for (const auto& [key, condition] : preconditions)
        {
            if (!world_model.has_state(key)) return false;
            if (!condition.evaluate(world_model, key)) return false;
        }
        return true;
    }

    void goap_action::apply_effects(world_state& world_model) const
    {
        for (const auto& [key, value] : effects)
        {
            world_model.set_state(key, value);
        }
    }
}