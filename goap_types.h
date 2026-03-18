//
// Created by Niall Ó Colmáin on 03/03/2026.
//

#ifndef IDAGOAP_GOAP_TYPES_H
#define IDAGOAP_GOAP_TYPES_H

#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace rida_goap
{
    class world_state;
    enum class predicate_op
    {
        Equal,
        NotEqual,
        Less,
        LessOrEqual,
        Greater,
        GreaterOrEqual
    };

    [[nodiscard]] inline std::string_view to_string(const predicate_op op) noexcept
    {
        switch (op)
        {
            case predicate_op::Equal:          return "==";
            case predicate_op::NotEqual:       return "!=";
            case predicate_op::Less:           return "<";
            case predicate_op::LessOrEqual:    return "<=";
            case predicate_op::Greater:        return ">";
            case predicate_op::GreaterOrEqual: return ">=";
            default:                           return "?";
        }
    }

    using state_value = std::variant<int, bool, float, std::string, std::vector<float>>;

    struct state_condition
    {
        state_value s_value;
        predicate_op predicate = predicate_op::Equal;

        state_condition() = default;

        state_condition(state_value value, const predicate_op comparison = predicate_op::Equal)
        : s_value(std::move(value)), predicate(comparison) {}

        [[nodiscard]] bool evaluate(const world_state &world_model, const std::string &key) const;

    private:
        template <typename T>

        [[nodiscard]] bool compare(const T& lhs, const T& rhs) const
        {
            if constexpr (std::is_same_v<T, bool> ||
                std::is_same_v<T, std::string> ||
                std::is_same_v<T, std::vector<float>>)
            {
                if (predicate != predicate_op::Equal && predicate != predicate_op::NotEqual)
                {
                    return false;
                }
            }

            switch (predicate)
            {
                case predicate_op::Equal:            return lhs == rhs;
                case predicate_op::NotEqual:         return lhs != rhs;
                case predicate_op::Less:             return lhs < rhs;
                case predicate_op::LessOrEqual:      return lhs <= rhs;
                case predicate_op::Greater:          return lhs > rhs;
                case predicate_op::GreaterOrEqual:   return lhs >= rhs;
                default:                             return false;
            }
        }
    };
}

namespace rida_goap::detail
{
    template<typename T>
    inline constexpr bool always_false_v = false;
}

template <>
struct std::hash<rida_goap::state_value>
{
    std::size_t operator()(const rida_goap::state_value& value) const noexcept
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
            else
            {
                static_assert(rida_goap::detail::always_false_v<T>,
                    "std::hash<state_value>: unhandled variant type — add a hash case");
                return 0;
            }
        }, value);
    }
};

#endif //IDAGOAP_GOAP_TYPES_H