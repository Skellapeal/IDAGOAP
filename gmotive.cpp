//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gmotive.h"

bool gmotive::is_satisfied(const gworld_model &world_model) const
{
    for (const auto& [key, value] : desired_state.get_states())
    {
        if (auto current = world_model.get_state(key); !current || *current != value)
        {
            return false;
        }
    }
    return true;
}
