//
// Created by Niall Ó Colmáin on 14/03/2026.
//

#include <gtest/gtest.h>
#include "plan_result.h"
#include "goap_action.h"

using namespace rida_goap;

class dummy_action : public goap_action
{
public:
    dummy_action() : goap_action("dummy", 1) {}
    action_status on_tick(float) override { return action_status::Succeeded; }
};

TEST(PlanResult, SuccessReturnsTrueOnSuccess)
{
    plan_result r;
    r.status = plan_status::Success;
    EXPECT_TRUE(r.success());
}

TEST(PlanResult, SuccessReturnsFalseOnFailure)
{
    plan_result r;
    r.status = plan_status::NoSolutionExists;
    EXPECT_FALSE(r.success());
}

TEST(PlanResult, IsTriviallySatisfiedOnEmptySuccessfulPlan)
{
    plan_result r;
    r.status = plan_status::Success;
    EXPECT_TRUE(r.is_trivially_satisfied());
}

TEST(PlanResult, IsTriviallySatisfiedFalseWhenActionsPresent)
{
    plan_result r;
    r.status = plan_status::Success;
    r.actions.push_back(std::make_shared<dummy_action>());
    EXPECT_FALSE(r.is_trivially_satisfied());
}

TEST(PlanResult, StatusStringIsNonEmptyForAllStatuses)
{
    for (const auto status : {
        plan_status::Success, plan_status::NoSolutionExists,
        plan_status::TimedOut, plan_status::DepthLimitReached,
        plan_status::NodeLimitReached })
    {
        plan_result r; r.status = status;
        EXPECT_FALSE(r.status_string().empty())
            << "Empty status_string for status " << static_cast<int>(status);
    }
}

TEST(PlanResult, StatusStringForSuccess)
{
    plan_result r;
    r.status = plan_status::Success;
    EXPECT_FALSE(r.status_string().empty());
}

TEST(PlanResult, HasNoActionsOnEmptyPlan)
{
    const plan_result r;
    EXPECT_TRUE(r.has_no_actions());
}
