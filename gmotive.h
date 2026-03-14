//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GMOTIVE_H
#define IDAGOAP_GMOTIVE_H
#include "gworld_model.h"


class gmotive
{
    gworld_model goal_state;
    int priority = 0;

public:
    explicit gmotive(gworld_model desired, const int priority = 0) : goal_state(std::move(desired)), priority(priority) {}

    [[nodiscard]] int get_priority() const { return priority; }
    [[nodiscard]] const gworld_model& get_goal_state() const { return goal_state; }
    [[nodiscard]] bool is_satisfied(const gworld_model &goal) const;
};


#endif //IDAGOAP_GMOTIVE_H