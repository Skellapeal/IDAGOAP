//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "motive.h"

bool motive::is_satisfied(const world_state &world_model) const
{
    return world_model.satisfies(goal_state);
}