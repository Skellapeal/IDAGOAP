//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "transposition_table.h"

namespace rida_goap
{
    std::optional<int> transposition_table::lookup(const world_state &state) const noexcept
    {
        const auto it = table.find(state);
        if (it == table.end())
        {
            return std::nullopt;
        }

        eviction_order.splice(eviction_order.begin(), eviction_order, it->second.second);
        return it->second.first;
    }

    void transposition_table::store(const world_state &state, const int cost) const noexcept
    {
        if (max_size == 0) return;

        if (const auto it = table.find(state); it != table.end())
        {
            it->second.first = cost;
            eviction_order.splice(eviction_order.begin(), eviction_order, it->second.second);
            return;
        }

        if (table.size() >= max_size && !eviction_order.empty())
        {
            table.erase(eviction_order.back());
            eviction_order.pop_back();
        }

        eviction_order.push_front(state);
        table.emplace(state, std::make_pair(cost, eviction_order.begin()));
    }

    void transposition_table::clear() const
    {
        for (auto it = table.begin(); it != table.end();)
        {
            it = table.erase(it);
        }
        eviction_order.clear();
    }

    size_t transposition_table::size() const noexcept
    {
        return table.size();
    }
}