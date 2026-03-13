#ifndef TRANSPOSITION_TABLE_H
#define TRANSPOSITION_TABLE_H

#include <list>
#include <limits>
#include "gworld_model.h"
#include <unordered_map>
#include <optional>

class gtranpos_table
{
    std::list<gworld_model> lru_order;  // Least Recently Used
    std::unordered_map<gworld_model, std::pair<int, std::list<gworld_model>::iterator>> table;
    size_t max_size = std::numeric_limits<size_t>::max();

public:
    void set_max_size(const size_t max) { max_size = max; }

    [[nodiscard]] std::optional<int> lookup(const gworld_model& state);
    void store(const gworld_model& state, int cost);
    void clear();
    [[nodiscard]] size_t size() const;
};

#endif // TRANSPOSITION_TABLE_H