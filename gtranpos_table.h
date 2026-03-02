#ifndef TRANSPOSITION_TABLE_H
#define TRANSPOSITION_TABLE_H

#include <limits>
#include "gworld_model.h"
#include <unordered_map>
#include <optional>

class gtranpos_table
{
    std::unordered_map<gworld_model, int> table;
    size_t max_size = std::numeric_limits<size_t>::max();

public:
    void set_max_size(const size_t max) { max_size = max; }
    [[nodiscard]] std::optional<int> lookup(const gworld_model& state) const;
    void store(const gworld_model& state, int cost);
    void clear();
    [[nodiscard]] size_t size() const;
};

#endif // TRANSPOSITION_TABLE_H