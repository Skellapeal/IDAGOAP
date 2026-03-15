//
// Created by Niall Ó Colmáin on 08/03/2026.
//

#ifndef IDAGOAP_HEURISTICS_H
#define IDAGOAP_HEURISTICS_H

#include "heuristic.h"
#include <cmath>
#include <memory>

namespace rida_goap
{
    ///@brief
    /// Uses dijkstra's heuristic (0). Correct but potentially slow
    /// use for testing or only when no better heuristic is available.
    class zero_heuristic : public heuristic
    {
    public:
        [[nodiscard]] int estimate(const world_state& world_model, const world_state& goal) const override
        {
            return 0;
        }
    };

    ///@brief
    /// Counts unsatisfied goals by the world model.
    /// This is the standard heuristic for GOAP
    class goal_count_heuristic : public heuristic
    {
    public:
        [[nodiscard]] int estimate(const world_state& world_model, const world_state& goal) const override
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

    ///@brief
    /// Euclidean distance between world and goal positions.
    /// Only meaningful for position navigation.
    /// Defaults to 0 if either world or goal lacks position key
    class euclidean_heuristic : public heuristic
    {
        std::string position_key;

    public:
        explicit euclidean_heuristic(std::string pos_key = "position") : position_key(std::move(pos_key)) {}

        [[nodiscard]] int estimate(const world_state& world_model, const world_state& goal) const override
        {
            const auto current_pos = world_model.get_position(position_key);
            const auto goal_pos = goal.get_position(position_key);

            if (!current_pos || !goal_pos)
            {
                return 0;
            }
            float distance = 0.0f;
            const size_t dims = std::min(current_pos->size(), goal_pos->size());

            for (size_t i = 0; i < dims; ++i)
            {
                const float diff = (*goal_pos)[i] - (*current_pos)[i];
                distance += diff * diff;
            }

            return static_cast<int>(std::ceil(std::sqrt(distance)));
        }
    };

    ///@brief
    /// Manhattan distance between world and goal positions.
    /// Ideal for grid-based movement without diagonals.
    class manhattan_heuristic : public heuristic
    {
        std::string position_key;

    public:
        explicit manhattan_heuristic(std::string pos_key = "position")
            : position_key(std::move(pos_key)) {}

        [[nodiscard]] int estimate(const world_state& world_model, const world_state& goal) const override
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

    ///@brief
    /// Weighted combination of multiple heuristics.
    /// Weights > 1.0 no longer guarantee optimal routing
    class composite_heuristic : public heuristic
    {
        std::vector<std::pair<std::shared_ptr<heuristic>, float>> sub_heuristics;

    public:
        void add_heuristic(std::shared_ptr<heuristic> h, float weight = 1.0f)
        {
            sub_heuristics.emplace_back(std::move(h), weight);
        }

        [[nodiscard]] int estimate(const world_state& world_model, const world_state& goal) const override
        {
            if (sub_heuristics.empty()) return 0;
            float total = 0.0f;

            for (const auto& [h, weight] : sub_heuristics)
            {
                total += weight * static_cast<float>(h->estimate(world_model, goal));
            }

            return static_cast<int>(total);
        }
    };
}

#endif //IDAGOAP_HEURISTICS_H