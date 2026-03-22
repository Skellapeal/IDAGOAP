//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_HEURISTIC_H
#define IDAGOAP_HEURISTIC_H
#include "world_state.h"

namespace rida_goap
{
    class heuristic
    {
    public:
        virtual ~heuristic() = default;

        // Force subclasses to implement estimate method
        [[nodiscard]] virtual int estimate(const world_state& world_model, const world_state& goal) const = 0;
    };
}

#endif //IDAGOAP_HEURISTIC_H