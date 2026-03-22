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
        std::string name;
        world_state goal_state;
        int priority = 0;
        bool persistent = false;

    public:
        explicit motive(std::string name, world_state desired, const int priority = 0)
            : name(std::move(name)), goal_state(std::move(desired)), priority(priority) {}

        [[nodiscard]] const std::string& get_name() const noexcept { return name; }
        [[nodiscard]] int get_priority() const noexcept { return priority; }
        [[nodiscard]] const world_state& get_goal_state() const noexcept { return goal_state; }
        [[nodiscard]] bool is_persistent() const noexcept { return persistent; }
        void set_persistent(const bool p) noexcept { persistent = p; }
        [[nodiscard]] bool is_satisfied(const world_state &world_model) const;

        void set_priority(const int new_priority) noexcept
        {
            priority = persistent && new_priority <= 0 ? 1 : new_priority;
        }
    };
}

#endif //IDAGOAP_MOTIVE_H