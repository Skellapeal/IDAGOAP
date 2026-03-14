//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gmotive.h"

bool gmotive::is_satisfied(const gworld_model &world_model) const
{
    return world_model.satisfies(goal_state);
}