//
// Created by Niall Ó Colmáin on 03/03/2026.
//

#ifndef IDAGOAP_GTYPES_H
#define IDAGOAP_GTYPES_H

#include <cassert>
#include <string>
#include <utility>
#include <variant>
#include <vector>

class world_state;

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
    gvalue value;
    gcomparison comparison = gcomparison::Equal;

    gcondition() = default;

    explicit gcondition(gvalue value, const gcomparison comparison = gcomparison::Equal)
    : value(std::move(value)), comparison(comparison) {}

    [[nodiscard]] bool evaluate(const world_state &world_model, const std::string &key) const;

private:
    template <typename T>

    [[nodiscard]] bool compare(const T& lhs, const T& rhs) const
    {
        if constexpr (std::is_same_v<T, bool> ||
            std::is_same_v<T, std::string> ||
            std::is_same_v<T, std::vector<float>>)
        {
            if (comparison != gcomparison::Equal && comparison != gcomparison::NotEqual)
            {
                return false;
            }
        }

        switch (comparison)
        {
            case gcomparison::Equal:            return lhs == rhs;
            case gcomparison::NotEqual:         return lhs != rhs;
            case gcomparison::Less:             return lhs < rhs;
            case gcomparison::LessOrEqual:      return lhs <= rhs;
            case gcomparison::Greater:          return lhs > rhs;
            case gcomparison::GreaterOrEqual:   return lhs >= rhs;
            default:                            return false;
        }
    }
};

template<>
struct std::hash<gvalue>
{
    std::size_t operator()(const gvalue& value) const noexcept
    {
        return std::visit([]<typename ValueType>(const ValueType& held_value) -> size_t
        {
            using T = std::decay_t<ValueType>;
            if constexpr (std::is_same_v<T, int>) return std::hash<int>{}(held_value);
            else if constexpr (std::is_same_v<T, bool>) return std::hash<bool>{}(held_value);
            else if constexpr (std::is_same_v<T, float>) return std::hash<float>{}(held_value);
            else if constexpr (std::is_same_v<T, std::string>) return std::hash<std::string>{}(held_value);
            else if constexpr (std::is_same_v<T, std::vector<float>>)
            {
                size_t seed = 0;
                for (const float f : held_value)
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