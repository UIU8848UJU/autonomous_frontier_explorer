#include <gtest/gtest.h>

#include "exploration_core/policy/exploration_policy.hpp"

namespace
{

class FakeExplorationPolicy final : public exploration_core::IExplorationPolicy
{
public:
    void reset() override
    {
        reset_called = true;
    }

    exploration_core::ExplorationDecision decide(
        const exploration_core::ExplorationObservation & observation) override
    {
        last_mapping_active = observation.mapping_active;
        return {};
    }

    void on_outcome(const exploration_core::ExplorationOutcome & outcome) override
    {
        last_outcome = outcome.type;
    }

    bool reset_called{false};
    bool last_mapping_active{false};
    exploration_core::ExplorationOutcomeType last_outcome{
        exploration_core::ExplorationOutcomeType::CANCELED};
};

}  // namespace

TEST(ExplorationCoreTest, PolicyProtocolCarriesObservationAndOutcome)
{
    FakeExplorationPolicy policy;
    policy.reset();

    exploration_core::ExplorationObservation observation;
    observation.mapping_active = true;
    EXPECT_EQ(policy.decide(observation).type, exploration_core::ExplorationDecisionType::WAIT);

    exploration_core::ExplorationOutcome outcome;
    outcome.type = exploration_core::ExplorationOutcomeType::NAVIGATION_SUCCEEDED;
    policy.on_outcome(outcome);

    EXPECT_TRUE(policy.reset_called);
    EXPECT_TRUE(policy.last_mapping_active);
    EXPECT_EQ(
        policy.last_outcome,
        exploration_core::ExplorationOutcomeType::NAVIGATION_SUCCEEDED);
}

