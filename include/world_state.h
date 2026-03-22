//
// Created by Niall Ó Colmáin on 16/02/2026.
//

#ifndef IDAGOAP_WORLD_STATE_H
#define IDAGOAP_WORLD_STATE_H

#include <algorithm>
#include <optional>
#include <ranges>
#include <unordered_map>
#include <variant>
#include "goap_types.h"

namespace rida_goap
{
    class world_state
    {
        std::unordered_map<std::string, state_value> states;

        template <typename T>
        [[nodiscard]] std::optional<T> get_as(const std::string& key) const
        {
            if (const auto value = get_state(key))
            {
                if (const auto* typed = std::get_if<T>(&*value)) return *typed;
            }
            return std::nullopt;
        }

    public:
        void set_state(const std::string& key, state_value value);
        [[nodiscard]] std::optional<state_value> get_state(const std::string& key) const noexcept;

        [[nodiscard]] bool has_state(const std::string& key) const noexcept;
        void remove_state(const std::string& key);

        [[nodiscard]] const std::unordered_map<std::string, state_value>&
        get_states() const noexcept;

        void set_int(const std::string& key, int value)
        {
            set_state(key, state_value{ value });
        }
        [[nodiscard]] std::optional<int> get_int(const std::string& key) const
        {
            return get_as<int>(key);
        }

        void set_bool(const std::string& key, bool value)
        {
            set_state(key, state_value{ value });
        }
        [[nodiscard]] std::optional<bool> get_bool(const std::string& key) const
        {
            return get_as<bool>(key);
        }

        void set_float(const std::string& key, float value)
        {
            set_state(key, state_value{ value });
        }
        [[nodiscard]] std::optional<float> get_float(const std::string& key) const
        {
            return get_as<float>(key);
        }

        void set_string(const std::string& key, std::string value)
        {
            set_state(key, state_value{ std::move(value) });
        }
        [[nodiscard]] std::optional<std::string> get_string(const std::string& key) const
        {
            return get_as<std::string>(key);
        }

        void set_position(const std::string& key, const float x, const float y, const float z = 0.0f)
        {
            set_state(key, state_value{ std::vector{ x, y, z } });
        }

        void set_position_2d(const std::string& key, const float x, const float y)
        {
            set_state(key, state_value{ std::vector{ x, y } });
        }

        [[nodiscard]] std::optional<std::vector<float>>get_position(const std::string& key) const
        {
            return get_as<std::vector<float>>(key);
        }

        void merge(const world_state& other);
        void merge_defaults(const world_state& other);

        [[nodiscard]] bool satisfies(const std::unordered_map<std::string, state_condition>& goal_conditions) const;
        [[nodiscard]] bool satisfies(const world_state& goal) const;
        [[nodiscard]] bool operator==(const world_state& other) const;
    };
}

template <>
struct std::hash<rida_goap::world_state>
{
    size_t operator()(const rida_goap::world_state& model) const noexcept
    {
        const auto& raw = model.get_states();

        std::vector<std::pair<std::string_view, const rida_goap::state_value*>> entries;
        entries.reserve(raw.size());

        for (const auto& [keys, values] : raw)
        {
            entries.emplace_back(keys, &values);
        }
        std::ranges::sort(entries, {}, &std::pair<std::string_view, const rida_goap::state_value*>::first);

        size_t seed = 0;
        const std::hash<rida_goap::state_value> value_hasher;

        for (const auto& [key, val_ptr] : entries)
        {
            const size_t key_hash   = hash<std::string_view>{}(key);
            const size_t value_hash = value_hasher(*val_ptr);
            seed ^= key_hash   + 0x9e3779b9U + (seed << 6) + (seed >> 2);
            seed ^= value_hash + 0x9e3779b9U + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

#endif //IDAGOAP_WORLD_STATE_H