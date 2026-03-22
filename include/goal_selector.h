//
// Created by Niall Ó Colmáin on 07/03/2026.
//

#ifndef IDAGOAP_GOAL_SELECTOR_H
#define IDAGOAP_GOAL_SELECTOR_H

#include <functional>
#include <vector>
#include <memory>
#include "motive.h"
#include "world_state.h"

namespace rida_goap
{
    class goal_selector
    {
    public:
        using utility_evaluator = std::function<float(const motive&, const world_state&)>;

    private:
        std::vector<std::shared_ptr<motive>> motives;
        utility_evaluator utility_fn;

    public:
        goal_selector() : utility_fn([](const motive& motive, const world_state&)
            { return static_cast<float>(motive.get_priority());}) {}

        void add_motive(std::shared_ptr<motive> motive);
        void remove_motive(const std::shared_ptr<motive>& motive);
        void clear_motives();

        [[nodiscard]] std::shared_ptr<motive> find_motive(std::string_view name) const;

        void set_utility_evaluator(utility_evaluator evaluator)
        {
            utility_fn = std::move(evaluator);
        }

        [[nodiscard]] std::shared_ptr<motive> select_goal(const world_state& world_model) const;

        void set_motive_priority(std::string_view motive_name, int new_priority) const;

        void satisfy_motive(std::string_view motive_name) const;

        [[nodiscard]] std::optional<world_state> select_goal_state(const world_state& world_model) const
        {
            const auto motive = select_goal(world_model);
            if (!motive) return std::nullopt;
            return motive->get_goal_state();
        }

        [[nodiscard]] std::vector<std::pair<std::shared_ptr<motive>, float>>
        evaluate_all_motives(const world_state& world_model) const;

        [[nodiscard]] const std::vector<std::shared_ptr<motive>>& get_motives() const noexcept
        { return motives; }
    };
}

#endif //IDAGOAP_GOAL_SELECTOR_H