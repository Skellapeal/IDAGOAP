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

    // Force subclasses to implement estimate method
    [[nodiscard]] virtual int estimate(const gworld_model& world_model, const gworld_model& goal) const = 0;
};


#endif //IDAGOAP_GHEURISTIC_H