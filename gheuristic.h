//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GHEURISTIC_H
#define IDAGOAP_GHEURISTIC_H
#include "gworld_model.h"

class gheuristic
{
public:
    virtual ~gheuristic() = default;

    [[nodiscard]] virtual int estimate(const gworld_model& world_model, const gworld_model& goal) const
    {
        int unsatisfied = 0;
        for (const auto& [key, goal_value] : goal.get_states())
        {
            if (const auto current_value = world_model.get_state(key);
                !current_value || *current_value != goal_value)
                ++unsatisfied;
        }
        return unsatisfied;
    }
};


#endif //IDAGOAP_GHEURISTIC_H