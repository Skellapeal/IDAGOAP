//
// Created by Niall Ó Colmáin on 07/03/2026.
//

#include "goal_selector.h"
#include <algorithm>
#include <limits>

namespace rida_goap
{
    void goal_selector::add_motive(std::shared_ptr<motive> motive) { motives.push_back(std::move(motive)); }
    void goal_selector::remove_motive(const std::shared_ptr<motive> &motive) { std::erase(motives, motive); }
    void goal_selector::clear_motives() { motives.clear(); }

    std::shared_ptr<motive> goal_selector::find_motive(std::string_view name) const
    {
        const auto it = std::ranges::find_if(motives, [&](const auto& m)
        {
            return m->get_name() == name;
        });
        return it != motives.end() ? *it : nullptr;
    }

    std::shared_ptr<motive> goal_selector::select_goal(const world_state &world_model) const
    {
        if (motives.empty()) return nullptr;

        std::shared_ptr<motive> highest_priority_motive = nullptr;
        float highest_utility = -std::numeric_limits<float>::infinity();

        for (const auto& motive : motives)
        {
            if (motive->get_priority() <= 0) continue;
            if (motive->is_satisfied(world_model)) continue;

            if (const float utility = utility_fn(*motive, world_model); utility > highest_utility)
            {
                highest_utility = utility;
                highest_priority_motive = motive;
            }
        }
        return highest_priority_motive;
    }

    void goal_selector::set_motive_priority(const std::string_view motive_name, const int new_priority) const
    {
        const auto m = find_motive(motive_name);
        if (m) m->set_priority(new_priority);
    }

    void goal_selector::satisfy_motive(const std::string_view motive_name) const
    {
        set_motive_priority(motive_name, 0);
    }

    std::vector<std::pair<std::shared_ptr<motive>, float> > goal_selector::evaluate_all_motives(const world_state &world_model) const
    {
        std::vector<std::pair<std::shared_ptr<motive>, float>> results;
        results.reserve(motives.size());

        for (const auto& motive : motives)
        {
            if (motive->get_priority() <= 0) continue;
            if (motive->is_satisfied(world_model)) continue;
            results.emplace_back(motive, utility_fn(*motive, world_model));
        }

        std::ranges::sort(results, [](const auto& a, const auto& b) { return a.second > b.second; });
        return results;
    }
}