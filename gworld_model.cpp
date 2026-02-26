//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gworld_model.h"

bool gworld_model::operator==(const gworld_model& other) const
{
    return states == other.states;
}

void gworld_model::set_state(const std::string &key, const int value)
{
    states[key] = value;
}

std::optional<int> gworld_model::get_state(const std::string &key) const
{
    if (const auto it = states.find(key); it != states.end())
    {
        return it->second;
    }
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

const std::unordered_map<std::string, int> & gworld_model::get_states() const
{
    return states;
}