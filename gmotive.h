//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GMOTIVE_H
#define IDAGOAP_GMOTIVE_H
#include "gworld_model.h"


class gmotive
{
    gworld_model desired_state;
    int priority = 0;

public:
    gmotive(gworld_model desired, int priority = 0) : desired_state(std::move(desired)), priority(priority) {}

    [[nodiscard]] int get_priority() const { return priority; }
    [[nodiscard]] const gworld_model& get_desired_state() const { return desired_state; }
    [[nodiscard]] bool is_satisfied(const gworld_model &world_model) const;
};


#endif //IDAGOAP_GMOTIVE_H