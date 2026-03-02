//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gheuristic.h"

int gheuristic::estimate(const gworld_model &world_model, const gworld_model &goal) const
{
    int h = 0;
    for (const auto& [key, goal_value] : goal.get_states())
    {
        if (auto current = world_model.get_state(key);
            !current || *current != goal_value)
        {
            ++h;
        }
    }
    return h;
}