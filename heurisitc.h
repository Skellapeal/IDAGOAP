//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GHEURISTIC_H
#define IDAGOAP_GHEURISTIC_H
#include "world_state.h"

class heurisitc
{
public:
    virtual ~heurisitc() = default;

    // Force subclasses to implement estimate method
    [[nodiscard]] virtual int estimate(const world_state& world_model, const world_state& goal) const = 0;
};


#endif //IDAGOAP_GHEURISTIC_H