//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GWORLD_MODEL_H
#define IDAGOAP_GWORLD_MODEL_H
#include <optional>
#include <variant>
#include "gaction.h"
#include "gtypes.h"

class gworld_model
{
    std::unordered_map<std::string, gvalue> states;

public:
    void set_state(const std::string& key, gvalue value);
    [[nodiscard]] std::optional<gvalue> get_state(const std::string& key) const;

    [[nodiscard]] bool has_state(const std::string& key) const;
    void remove_state(const std::string& key);

    [[nodiscard]] const std::unordered_map<std::string, gvalue>& get_states() const;

    void set_int(const std::string& key, int value) { set_state(key, gvalue{value}); }
    [[nodiscard]] std::optional<int> get_int(const std::string& key) const;

    void set_bool(const std::string& key, bool value) { set_state(key, gvalue{value}); }
    [[nodiscard]] std::optional<bool> get_bool(const std::string& key) const;

    void set_float(const std::string& key, float value) { set_state(key, gvalue{value}); }
    [[nodiscard]]std::optional<float> get_float(const std::string &key) const;

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
            const size_t value_hash = hash<gvalue>{}(value);

            seed ^= key_hash + value_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
            seed ^= value_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
#endif //IDAGOAP_GWORLD_MODEL_H