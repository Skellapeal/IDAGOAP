//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "world_state.h"

bool world_state::operator==(const world_state& other) const { return states == other.states; }
void world_state::set_state(const std::string &key, gvalue value) { states[key] = std::move(value); }

std::optional<gvalue> world_state::get_state(const std::string &key) const
{
    if (const auto it = states.find(key); it != states.end())
    {
        return it->second;
    }
    return std::nullopt;
}

bool world_state::has_state(const std::string& key) const { return states.contains(key); }
void world_state::remove_state(const std::string &key) { states.erase(key); }
const std::unordered_map<std::string, gvalue> & world_state::get_states() const { return states; }

std::optional<int> world_state::get_int(const std::string &key) const
{
    if (const auto value = get_state(key))
    {
        if (const auto* int_ptr = std::get_if<int>(&*value))
        {
            return *int_ptr;
        }
    }
    return std::nullopt;
}

std::optional<bool> world_state::get_bool(const std::string &key) const
{
    if (const auto value = get_state(key))
    {
        if (const auto* bool_ptr = std::get_if<bool>(&*value))
        {
            return *bool_ptr;
        }
    }
    return std::nullopt;
}

std::optional<float> world_state::get_float(const std::string &key) const
{
    if (const auto value = get_state(key))
    {
        if (const auto* float_ptr = std::get_if<float>(&*value))
        {
            return *float_ptr;
        }
    }
    return std::nullopt;
}

std::optional<std::string> world_state::get_string(const std::string& key) const
{
    if (const auto value = get_state(key))
    {
        if (const auto* str_ptr = std::get_if<std::string>(&*value))
        {
            return *str_ptr;
        }
    }
    return std::nullopt;
}

std::optional<std::vector<float>> world_state::get_position(const std::string& key) const
{
    if (const auto value = get_state(key))
    {
        if (const auto* vec_ptr = std::get_if<std::vector<float>>(&*value))
        {
            return *vec_ptr;
        }
    }
    return std::nullopt;
}

void world_state::merge(const world_state &other)
{
    for (const auto& [key, value] : other.get_states())
    {
        states[key] = value;
    }
}

bool world_state::satisfies(const world_state &goal) const
{
    return !std::ranges::any_of(goal.get_states(),
        [&](const std::pair<const std::string, gvalue>& goal_state)
        {
            const auto& [key, value] = goal_state;
            const auto start_value = get_state(key);
            return !start_value || *start_value != value;
        });
}
