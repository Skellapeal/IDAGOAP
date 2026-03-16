//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_MOTIVE_H
#define IDAGOAP_MOTIVE_H
#include "world_state.h"

namespace rida_goap
{
    class motive
    {
        world_state goal_state;
        int priority = 0;

    public:
        explicit motive(world_state desired, const int priority = 0)
            : goal_state(std::move(desired)), priority(priority) {}

        [[nodiscard]] int get_priority() const noexcept { return priority; }
        [[nodiscard]] const world_state& get_goal_state() const noexcept { return goal_state; }
        [[nodiscard]] bool is_satisfied(const world_state &world_model) const;
    };
}

#endif //IDAGOAP_MOTIVE_H