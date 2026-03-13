//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gtranpos_table.h"

std::optional<int> gtranpos_table::lookup(const gworld_model &state)
{
    const auto it = table.find(state);
    if (it == table.end())
    {
        return std::nullopt;
    }

    lru_order.splice(lru_order.begin(), lru_order, it->second.second);
    return it->second.first;
}

void gtranpos_table::store(const gworld_model &state, const int cost)
{
    if (const auto it = table.find(state); it != table.end())
    {
        it->second.first = cost;
        lru_order.splice(lru_order.begin(), lru_order, it->second.second);
        return;
    }

    if (table.size() >= max_size && !lru_order.empty())
    {
        table.erase(lru_order.back());
        lru_order.pop_back();
    }

    lru_order.push_front(state);
    table.emplace(state, std::make_pair(cost, lru_order.begin()));
}

void gtranpos_table::clear()
{
    table.clear();
    lru_order.clear();
}

size_t gtranpos_table::size() const
{
    return table.size();
}
