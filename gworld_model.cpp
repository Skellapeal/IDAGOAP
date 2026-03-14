//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "gworld_model.h"

bool gworld_model::operator==(const gworld_model& other) const { return states == other.states; }
void gworld_model::set_state(const std::string &key, gvalue value) { states[key] = std::move(value); }

std::optional<gvalue> gworld_model::get_state(const std::string &key) const
{
    if (const auto it = states.find(key); it != states.end())
    {
        return it->second;
    }
    return std::nullopt;
}

bool gworld_model::has_state(const std::string& key) const { return states.contains(key); }
void gworld_model::remove_state(const std::string &key) { states.erase(key); }
const std::unordered_map<std::string, gvalue> & gworld_model::get_states() const { return states; }

std::optional<int> gworld_model::get_int(const std::string &key) const
{
    if (const auto value = get_state(key))
    {
        if (const auto* int_value = std::get_if<int>(&*value))
        {
            return *int_value;
        }
    }
    return std::nullopt;
}

std::optional<bool> gworld_model::get_bool(const std::string &key) const
{
    if (const auto value = get_state(key))
    {
        if (const auto* bool_val = std::get_if<bool>(&*value))
        {
            return *bool_val;
        }
    }
    return std::nullopt;
}

std::optional<float> gworld_model::get_float(const std::string &key) const
{
    if (const auto value = get_state(key))
    {
        if (const auto* float_val = std::get_if<float>(&*value))
        {
            return *float_val;
        }
    }
    return std::nullopt;
}

std::optional<std::string> gworld_model::get_string(const std::string& key) const
{
    if (const auto value = get_state(key))
    {
        if (const auto* str_val = std::get_if<std::string>(&*value))
        {
            return *str_val;
        }
    }
    return std::nullopt;
}

std::optional<std::vector<float>> gworld_model::get_position(const std::string& key) const
{
    if (const auto value = get_state(key))
    {
        if (const auto* vec_val = std::get_if<std::vector<float>>(&*value))
        {
            return *vec_val;
        }
    }
    return std::nullopt;
}

void gworld_model::merge(const gworld_model &other)
{
    for (const auto& [key, value] : other.get_states())
    {
        states[key] = value;
    }
}

bool gworld_model::satisfies(const gworld_model &goal) const
{
    return !std::ranges::any_of(goal.get_states(),
        [&](const std::pair<std::string, gvalue>& goal_state)
        {
            const auto& [key, value] = goal_state;
            const auto start_value = goal.get_state(key);
            return !start_value || start_value != value;
        });
}
