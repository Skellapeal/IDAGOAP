#ifndef TRANSPOSITION_TABLE_H
#define TRANSPOSITION_TABLE_H

#include <list>
#include <limits>
#include "world_state.h"
#include <unordered_map>
#include <optional>

namespace rida_goap
{
    class transposition_table
    {
        std::list<world_state> eviction_order;  // Least Recently Used
        std::unordered_map<world_state, std::pair<int, std::list<world_state>::iterator>> table;
        size_t max_size = std::numeric_limits<size_t>::max();

    public:
        void set_max_size(const size_t max) { max_size = max; }

        [[nodiscard]] std::optional<int> lookup(const world_state& state);
        void store(const world_state& state, int cost);
        void clear();
        [[nodiscard]] size_t size() const;
    };
}

#endif // TRANSPOSITION_TABLE_H