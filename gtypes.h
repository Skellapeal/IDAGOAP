//
// Created by Niall Ó Colmáin on 03/03/2026.
//

#ifndef IDAGOAP_GTYPES_H
#define IDAGOAP_GTYPES_H

#pragma once
#include <string>
#include <variant>
#include <vector>

class gworld_model;

enum class gcomparison
{
    Equal,
    NotEqual,
    Less,
    LessOrEqual,
    Greater,
    GreaterOrEqual
};

using gvalue = std::variant<int, bool, float, std::string, std::vector<float>>;

struct gcondition
{
    std::string key;
    gvalue value;
    gcomparison comparison = gcomparison::Equal;

    [[nodiscard]] bool evaluate(const gworld_model& world_model) const;

private:
    template <typename T>
    bool compare(const T& left_hand_side, const T& right_hand_side) const
    {
        switch (comparison)
        {
            case gcomparison::Equal:
                return left_hand_side == right_hand_side;
            case gcomparison::NotEqual:
                return left_hand_side != right_hand_side;
            case gcomparison::Less:
                return left_hand_side < right_hand_side;
            case gcomparison::LessOrEqual:
                return left_hand_side <= right_hand_side;
            case gcomparison::Greater:
                return left_hand_side > right_hand_side;
            case gcomparison::GreaterOrEqual:
                return left_hand_side >= right_hand_side;
        }
        return false;
    }

    friend struct std::hash<gvalue>;
};

template<>
struct std::hash<gvalue>
{
    std::size_t operator()(const gvalue& value) const noexcept
    {
        return std::visit([]<typename T0>(const T0& new_value) -> size_t
        {
            using T = std::decay_t<T0>;
            if constexpr (std::is_same_v<T, int>)
            {
                return std::hash<int>{}(new_value);
            }
            else if constexpr (std::is_same_v<T, bool>)
            {
                return std::hash<bool>{}(new_value);
            }
            else if constexpr (std::is_same_v<T, float>)
            {
                return std::hash<float>{}(new_value);
            }
            else if constexpr (std::is_same_v<T, std::string>)
            {
                return std::hash<std::string>{}(new_value);
            }
            else if constexpr (std::is_same_v<T, std::vector<float>>)
            {
                size_t seed = 0;
                for (const float f : new_value)
                {
                    seed ^= std::hash<float>{}(f) + 0x9e3779b9U + (seed << 6) + (seed >> 2);
                }
                return seed;
            }
            return 0;
        }, value);
    }
};

#endif //IDAGOAP_GTYPES_H