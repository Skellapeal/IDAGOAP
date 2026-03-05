//
// Created by Niall Ó Colmáin on 03/03/2026.
//

#ifndef IDAGOAP_GTYPES_H
#define IDAGOAP_GTYPES_H

#pragma once
#include <string>
#include <variant>

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

using gvalue = std::variant<int, bool, float>;

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
};

#endif //IDAGOAP_GTYPES_H