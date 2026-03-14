//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GMOTIVE_H
#define IDAGOAP_GMOTIVE_H
#include "world_state.h"


class motive
{
    world_state goal_state;
    int priority = 0;

public:
    explicit motive(world_state desired, const int priority = 0) : goal_state(std::move(desired)), priority(priority) {}

    [[nodiscard]] int get_priority() const { return priority; }
    [[nodiscard]] const world_state& get_goal_state() const { return goal_state; }
    [[nodiscard]] bool is_satisfied(const world_state &goal) const;
};


#endif //IDAGOAP_GMOTIVE_H