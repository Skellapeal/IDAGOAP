//
// Created by ismis on 08/03/2026.
//

#ifndef IDAGOAP_GHEURISTIC_LIBRARY_H
#define IDAGOAP_GHEURISTIC_LIBRARY_H

#include "gheuristic.h"
#include <cmath>

class dijkstras_heuristic : public gheuristic
{
public:
    [[nodiscard]] int estimate(const gworld_model& world_model, const gworld_model& goal) const override
    {
        return 0;
    }
};

class goal_count_heuristic : public gheuristic
{
public:
    [[nodiscard]] int estimate(const gworld_model& world_model, const gworld_model& goal) const override
    {
        int unsatisfied = 0;

        for (const auto& [key, goal_value] : goal.get_states())
        {
            if (const auto current_value = world_model.get_state(key))
            {
                if (*current_value != goal_value)
                {
                    unsatisfied++;
                }
            }
            else
            {
                unsatisfied++;
            }
        }
        return unsatisfied;
    }
};

class euclidean_heuristic : public gheuristic
{
    std::string position_key;

public:
    explicit euclidean_heuristic(std::string pos_key = "position") : position_key(std::move(pos_key)) {}

    [[nodiscard]] int estimate(const gworld_model& world_model, const gworld_model& goal) const override
    {
        const auto current_pos = world_model.get_position(position_key);
        const auto goal_pos = goal.get_position(position_key);

        if (!current_pos || !goal_pos) return 0;

        float distance = 0.0f;
        const size_t dims = std::min(current_pos->size(), goal_pos->size());

        for (size_t i = 0; i < dims; ++i)
        {
            const float diff = (*goal_pos)[i] - (*current_pos)[i];
            distance += diff * diff;
        }

        return static_cast<int>(std::sqrt(distance));
    }
};

class manhattan_heuristic : public gheuristic
{
    std::string position_key;

public:
    explicit manhattan_heuristic(std::string pos_key = "position")
        : position_key(std::move(pos_key)) {}

    [[nodiscard]] int estimate(const gworld_model& world_model, const gworld_model& goal) const override
    {
        const auto current_pos = world_model.get_position(position_key);
        const auto goal_pos = goal.get_position(position_key);

        if (!current_pos || !goal_pos) return 0;

        float distance = 0.0f;
        const size_t dims = std::min(current_pos->size(), goal_pos->size());

        for (size_t i = 0; i < dims; ++i)
        {
            distance += std::abs((*goal_pos)[i] - (*current_pos)[i]);
        }

        return static_cast<int>(distance);
    }
};

class composite_heuristic : public gheuristic
{
    std::vector<std::pair<std::shared_ptr<gheuristic>, float>> heuristics;

public:
    void add_heuristic(std::shared_ptr<gheuristic> h, float weight = 1.0f)
    {
        heuristics.emplace_back(std::move(h), weight);
    }

    [[nodiscard]] int estimate(const gworld_model& world_model, const gworld_model& goal) const override
    {
        float total = 0.0f;

        for (const auto& [heuristic, weight] : heuristics)
        {
            total += weight * static_cast<float>(heuristic->estimate(world_model, goal));
        }

        return static_cast<int>(total);
    }
};

#endif //IDAGOAP_GHEURISTIC_LIBRARY_H