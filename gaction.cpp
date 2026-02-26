//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gaction.h"

#include "gworld_model.h"

bool gaction::check_preconditions(const gworld_model* world_model) const
{
    for (const auto& [key, value] : preconditions)
    {
        if (world_model->get_state(key) != value)
        {
            return false;
        }
    }
    return true;
}

void gaction::apply_effects(gworld_model* world_model) const
{
    for (const auto& [key, value] : effects)
    {
        world_model->set_state(key, value);
    }
}