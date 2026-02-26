//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GWORLD_MODEL_H
#define IDAGOAP_GWORLD_MODEL_H
#include <optional>
#include "gaction.h"

class gworld_model
{
    std::unordered_map<std::string, int> states;

public:
    void set_state(const std::string& key, int value);
    [[nodiscard]] std::optional<int> get_state(const std::string& key) const;
    [[nodiscard]] bool has_state(const std::string& key) const;
    void remove_state(const std::string& key);

    [[nodiscard]] const std::unordered_map<std::string, int>& get_states() const;

    [[nodiscard]] bool operator==(const gworld_model &other) const;
};

template <>
struct std::hash<gworld_model>
{
    size_t operator()(const gworld_model& model) const noexcept
    {
        size_t seed = 0;
        for (const auto& [key, value] : model.get_states())
        {
            const size_t key_hash = hash<string>{}(key);
            const size_t value_hash = hash<int>{}(value);

            seed ^= key_hash + value_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= value_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
#endif //IDAGOAP_GWORLD_MODEL_H