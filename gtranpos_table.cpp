//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gtranpos_table.h"

std::optional<int> gtranpos_table::lookup(const gworld_model &state) const
{
    if (const auto it = table.find(state); it != table.end())
    {
        return it->second;
    }
    return std::nullopt;
}

void gtranpos_table::store(const gworld_model &state, const int cost)
{
    if (table.size() >= max_size)
    {
        table.clear();
    }

    table[state] = cost;
}

void gtranpos_table::clear()
{
    table.clear();
}

size_t gtranpos_table::size() const
{
    return table.size();
}
