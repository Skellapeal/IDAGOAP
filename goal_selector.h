//
// Created by ismis on 07/03/2026.
//

#ifndef IDAGOAP_GOAL_SELECTOR_H
#define IDAGOAP_GOAL_SELECTOR_H

#include <functional>
#include <vector>
#include <memory>
#include "gmotive.h"
#include "gworld_model.h"

class goal_selector
{
public:
    using utility_evaluator = std::function<float(const gmotive&, const gworld_model&)>;

private:
    std::vector<std::shared_ptr<gmotive>> motives;
    utility_evaluator evaluator_utility;

public:
    goal_selector()
    {
        evaluator_utility = [](const gmotive& motive, const gworld_model&)
        {
            return static_cast<float>(motive.get_priority());
        };
    }

    void add_motive(std::shared_ptr<gmotive> motive);
    void remove_motive(const std::shared_ptr<gmotive>& motive);
    void clear_motives();

    void set_utility_evaluator(utility_evaluator evaluator)
    {
        evaluator_utility = std::move(evaluator);
    }

    [[nodiscard]] std::shared_ptr<gmotive> select_goal(const gworld_model& world_model) const;
    [[nodiscard]] std::optional<gworld_model> select_goal_state(const gworld_model& world_model) const
    {
        const auto motive = select_goal(world_model);
        if (!motive) return std::nullopt;
        return motive->get_goal_state();
    }
    [[nodiscard]] std::vector<std::pair<std::shared_ptr<gmotive>, float>> evaluate_all_motives(const gworld_model& world_model) const;
    [[nodiscard]] const std::vector<std::shared_ptr<gmotive>>& get_motives() const { return motives; }
};

#endif //IDAGOAP_GOAL_SELECTOR_H