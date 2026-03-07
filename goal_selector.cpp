//
// Created by ismis on 07/03/2026.
//

#include "goal_selector.h"
#include <algorithm>
#include <limits>

void goal_selector::add_motive(std::shared_ptr<gmotive> motive) { motives.push_back(std::move(motive)); }
void goal_selector::remove_motive(const std::shared_ptr<gmotive> &motive) { std::erase(motives, motive); }
void goal_selector::clear_motives() { motives.clear(); }

std::shared_ptr<gmotive> goal_selector::select_goal(const gworld_model &world_model) const
{
    if (motives.empty()) return nullptr;

    std::shared_ptr<gmotive> best_motive = nullptr;
    float best_utility = -std::numeric_limits<float>::infinity();

    for (const auto& motive : motives)
    {
        if (motive->is_satisfied(world_model)) continue;

        if (const float utility = evaluator_utility(*motive, world_model); utility > best_utility)
        {
            best_utility = utility;
            best_motive = motive;
        }
    }
    return best_motive;
}

std::vector<std::pair<std::shared_ptr<gmotive>, float> > goal_selector::evaluate_all_motives(const gworld_model &world_model) const
{
    std::vector<std::pair<std::shared_ptr<gmotive>, float>> results;
    results.reserve(motives.size());

    for (const auto& motive : motives)
    {
        const float utility = evaluator_utility(*motive, world_model);
        results.emplace_back(motive, utility);
    }

    std::ranges::sort(results, [](const auto& a, const auto& b) { return a.second > b.second; });
    return results;
}
