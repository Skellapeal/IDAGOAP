//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#include "world_state.h"

namespace rida_goap
{
    bool world_state::operator==(const world_state& other) const { return states == other.states; }
    void world_state::set_state(const std::string& key, state_value value) { states[key] = std::move(value); }

    std::optional<state_value> world_state::get_state(const std::string& key) const noexcept
    {
        if (const auto it = states.find(key); it != states.end()) return it->second;
        return std::nullopt;
    }

    bool world_state::has_state(const std::string& key) const noexcept
    { return states.contains(key); }

    void world_state::remove_state(const std::string& key) { states.erase(key); }

    const std::unordered_map<std::string, state_value>&world_state::get_states() const noexcept
    { return states; }

    void world_state::merge(const world_state& other)
    {
        for (const auto& [key, value] : other.get_states()) states[key] = value;
    }

    void world_state::merge_defaults(const world_state& other)
    {
        for (const auto& [key, value] : other.get_states())
        {
            if (!has_state(key)) set_state(key, value);
        }
    }

    bool world_state::satisfies(
        const std::unordered_map<std::string, state_condition>& goal_conditions) const
    {
        return std::ranges::all_of(goal_conditions,[this](const auto& entry)
            {
                const auto& [key, condition] = entry;
                return condition.evaluate(*this, key);
            });
    }

    bool world_state::satisfies(const world_state& goal) const
    {
        return !std::ranges::any_of(goal.get_states(),
            [&](const std::pair<const std::string, state_value>& goal_state)
            {
                const auto& [key, value] = goal_state;
                const auto start_value   = get_state(key);
                return !start_value || *start_value != value;
            });
    }
}