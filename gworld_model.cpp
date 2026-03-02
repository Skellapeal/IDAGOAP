//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gworld_model.h"

bool gworld_model::operator==(const gworld_model& other) const
{
    return states == other.states;
}

void gworld_model::set_state(const std::string &key, const gvalue value)
{
    states[key] = value;
}

std::optional<gvalue> gworld_model::get_state(const std::string &key) const
{
    if (const auto it = states.find(key); it != states.end())
    {
        return it->second;
    }
    return std::nullopt;
}

std::optional<int> gworld_model::get_int(const std::string &key) const
{
    const auto it = states.find(key);
    if (it == states.end()) return std::nullopt;

    if (const auto pval = std::get_if<int>(&it->second)) return *pval;
    return std::nullopt;
}

std::optional<bool> gworld_model::get_bool(const std::string &key) const
{
    const auto it = states.find(key);
    if (it == states.end()) return std::nullopt;

    if (const auto pval = std::get_if<bool>(&it->second)) return *pval;
    return std::nullopt;
}

std::optional<float> gworld_model::get_float(const std::string &key) const
{
    const auto it = states.find(key);
    if (it == states.end()) return std::nullopt;

    if (const auto pval = std::get_if<float>(&it->second)) return *pval;
    return std::nullopt;
}

bool gworld_model::has_state(const std::string &key) const
{
    return states.contains(key);
}

void gworld_model::remove_state(const std::string &key)
{
    states.erase(key);
}

const std::unordered_map<std::string, gvalue> & gworld_model::get_states() const
{
    return states;
}

inline std::size_t hash_gvalue(const gvalue& v)
{
    std::size_t seed = std::hash<std::size_t>{}(v.index());

    std::visit([&]<typename T0>(const T0& value)
    {
        using T = std::decay_t<T0>;
        const std::size_t value_hash = std::hash<T>{}(value);
        seed ^= value_hash + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }, v);

    return seed;
}