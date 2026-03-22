//
// Created by Niall Ó Colmáin on 05/03/2026.
//

#include "goap_types.h"
#include "world_state.h"

namespace rida_goap
{
    bool state_condition::evaluate(const world_state& world_model, const std::string& key) const
    {
        const auto world_value = world_model.get_state(key);
        if (!world_value) return false;

        return std::visit([this, &world_value]<typename T0>(const T0& cond_value) -> bool
        {
            using T = std::decay_t<T0>;

            if (const auto* world_val = std::get_if<T>(&*world_value))
            {
                return compare(*world_val, cond_value);
            }
            return false;
        }, s_value);
    }
}