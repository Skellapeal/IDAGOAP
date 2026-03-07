//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gmotive.h"
#include <algorithm>

bool gmotive::is_satisfied(const gworld_model &world_model) const
{
    return std::ranges::all_of(desired_state.get_states(),
        [&world_model](const auto& state_entry)
        {
            const auto& [key, desired_value] = state_entry;
            const auto actual_value = world_model.get_state(key);
            return actual_value && *actual_value == desired_value;
        }
    );
}