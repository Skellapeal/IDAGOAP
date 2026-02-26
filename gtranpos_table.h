#ifndef TRANSPOSITION_TABLE_H
#define TRANSPOSITION_TABLE_H

#include "gworld_model.h"
#include <unordered_map>
#include <optional>

class gtranpos_table
{
    std::unordered_map<gworld_model, int> table;

public:
    [[nodiscard]] std::optional<int> lookup(const gworld_model& state) const;
    void store(const gworld_model& state, int cost);
    void clear();
    [[nodiscard]] size_t size() const;
};

#endif // TRANSPOSITION_TABLE_H