//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_GWORLD_MODEL_H
#define IDAGOAP_GWORLD_MODEL_H
#include <algorithm>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <variant>
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

    void set_string(const std::string& key, std::string value) { set_state(key, gvalue{ std::move(value) }); }
    [[nodiscard]] std::optional<std::string> get_string(const std::string& key) const;

    void set_position(const std::string& key, const float x, const float y, const float z = 0.0f) { set_state(key, gvalue{std::vector{x, y, z}}); }
    [[nodiscard]] std::optional<std::vector<float>> get_position(const std::string& key) const;

    void merge(const gworld_model& other);

    [[nodiscard]] bool operator==(const gworld_model &other) const;
};

template <>
struct std::hash<gworld_model>
{
    size_t operator()(const gworld_model& model) const noexcept
    {
        const auto& raw = model.get_states();

        std::vector<std::string_view> keys;
        keys.reserve(raw.size());

        for (const auto &k: raw | views::keys)
        {
            keys.emplace_back(k);
        }
        ranges::sort(keys);

        size_t seed = 0;
        constexpr std::hash<gvalue> value_hasher;

        for (const auto& key : keys)
        {
            const size_t key_hash = hash<std::string_view>{}(key);
            const size_t value_hash = value_hasher(raw.at(std::string(key)));

            seed ^= key_hash + 0x9e3779b9U + (seed << 6) + (seed >> 2);
            seed ^= value_hash + 0x9e3779b9U + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};
#endif //IDAGOAP_GWORLD_MODEL_H