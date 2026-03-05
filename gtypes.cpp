//
// Created by Niall Ó Colmáin on 05/03/2026.
//

#include "gtypes.h"
#include "gworld_model.h"

bool gcondition::evaluate(const gworld_model &world_model) const
{
    const auto current = world_model.get_state(key);

    if (!current) return false;
    if (current->index() != value.index()) return false;

    if (const auto* this_int = std::get_if<int>(&*current))
    {
        const auto* other_int = std::get_if<int>(&value);
        return compare(*this_int, *other_int);
    }

    if (const auto* this_float = std::get_if<float>(&*current))
    {
        const auto* other_float = std::get_if<float>(&value);
        return compare(*this_float, *other_float);
    }

    if (const auto* this_bool = std::get_if<bool>(&*current))
    {
        const auto* other_bool = std::get_if<bool>(&value);
        return compare(*this_bool, *other_bool);
    }

    return false;
}
