#include <gtest/gtest.h>
#include <limits>
#include "height_jump_mppi_critic/evaluator.hpp"

using namespace height_jump;

TEST(Evaluator, BinaryThresholdHasNoPreferenceWithinPassableRange)
{
  Config cfg{0.1, 0.25f};
  auto check = [&](float height) {
      return evaluate(0, 0, cfg, [height](double x, double) {
          return Sample{SampleState::Known, x >= 0 ? height : 0.0f};
        }, [](double, double) {return false;});
    };
  EXPECT_EQ(check(0).decision, Decision::Pass);
  EXPECT_EQ(check(0.05f).decision, Decision::Pass);
  EXPECT_EQ(check(0.20f).decision, Decision::Pass);
  EXPECT_EQ(check(0.25f).decision, Decision::HeightJump);
  EXPECT_EQ(check(0.60f).decision, Decision::HeightJump);
}

TEST(Evaluator, SparseHolesAreToleratedButThreeRequireExemption)
{
  for (int missing : {0, 1, 2, 3, 9}) {
    for (bool allow : {false, true}) {
      int index = 0;
      auto result = evaluate(0, 0, Config{}, [&](double, double) {
          return Sample{index++ < missing ? SampleState::Unknown : SampleState::Known, 0};
        }, [allow](double, double) {return allow;});
      EXPECT_EQ(result.decision, missing >= 3 && !allow ? Decision::Unknown : Decision::Pass);
      EXPECT_EQ(result.unknown_count, missing);
      EXPECT_EQ(index, 9);
    }
  }
}

TEST(Evaluator, KnownDangerTakesPriorityOverStartupUnknown)
{
  int index = 0;
  auto result = evaluate(0, 0, Config{}, [&](double, double) {
      const int i = index++;
      return Sample{i < 3 ? SampleState::Unknown : SampleState::Known, i == 3 ? 0.6f : 0};
    }, [](double, double) {return true;});
  EXPECT_EQ(result.decision, Decision::HeightJump);
}

TEST(Evaluator, OutsideMapCannotBeExemptedAndNonFiniteHeightsAreUnknown)
{
  auto allow = [](double, double) {return true;};
  EXPECT_EQ(evaluate(0, 0, Config{}, [](double, double) {
      return Sample{SampleState::Outside, 0};
    }, allow).decision, Decision::Outside);
  for (float z : {std::numeric_limits<float>::quiet_NaN(),
      std::numeric_limits<float>::infinity()})
  {
    EXPECT_EQ(evaluate(0, 0, Config{}, [z](double, double) {
        return Sample{SampleState::Known, z};
      }, [](double, double) {return false;}).decision, Decision::Unknown);
  }
}

TEST(StartupRegion, FixedCenterAndEachUnknownSampleMustStayInside)
{
  StartupRegion region;
  EXPECT_FALSE(region.contains(5, -2, 0.7));
  EXPECT_TRUE(region.capture(5, -2));
  EXPECT_TRUE(region.capture(8, -2));
  EXPECT_TRUE(region.contains(5.6, -2, 0.7));
  EXPECT_FALSE(region.contains(8, -2, 0.7));
  auto unknown = [](double, double) {return Sample{SampleState::Unknown, 0};};
  auto allow = [&](double x, double y) {return region.contains(x, y, 0.7);};
  EXPECT_EQ(evaluate(5.5, -2, Config{}, unknown, allow).decision, Decision::Pass);
  EXPECT_EQ(evaluate(5.68, -2, Config{}, unknown, allow).decision, Decision::Unknown);
  EXPECT_EQ(evaluate(5.72, -2, Config{}, unknown, allow).decision, Decision::Unknown);
  StartupRegion new_instance;
  EXPECT_TRUE(new_instance.capture(8, -2));
  EXPECT_TRUE(new_instance.contains(8, -2, 0.7));
}

TEST(Evaluator, RejectsInvalidCoordinatesAndConfiguration)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_THROW((Config{0, 0.3f}.validate()), std::invalid_argument);
  EXPECT_THROW((Config{-0.1, 0.3f}.validate()), std::invalid_argument);
  EXPECT_THROW((Config{0.1, 0}.validate()), std::invalid_argument);
  EXPECT_THROW((Config{nan, 0.3f}.validate()), std::invalid_argument);
  StartupRegion region;
  EXPECT_FALSE(region.capture(nan, 0));
  EXPECT_EQ(evaluate(nan, 0, Config{}, [](double, double) {
      return Sample{SampleState::Known, 0};
    }, [](double, double) {return true;}).decision, Decision::Invalid);
}

TEST(Evaluator, TraversableUnknownDoesNotNeedStartupExemption)
{
  int exemptions = 0;
  const Config cfg{0.1, 0.3f, true};
  const auto unknown = [](double, double) {return Sample{SampleState::Unknown, 0};};
  const auto result = evaluate(0, 0, cfg, unknown,
    [&](double, double) {++exemptions; return true;});
  EXPECT_EQ(result.decision, Decision::Pass);
  EXPECT_EQ(result.unknown_count, 9);
  EXPECT_FLOAT_EQ(result.observed_jump, 0);
  EXPECT_EQ(exemptions, 0);
}

TEST(Evaluator, PreparedNeighborhoodPreservesStatisticsAndRejectionPriority)
{
  std::array<Sample, 9> samples;
  samples.fill({SampleState::Known, 0});
  samples[1] = {SampleState::Unknown, 0};
  samples[2] = {SampleState::Outside, 0};
  samples[8] = {SampleState::Known, 0.3f};
  for (bool unknown_pass : {false, true}) {
    const auto result = evaluateNeighborhood(0, 0, Config{0.1, 0.3f, unknown_pass}, samples,
      [](double, double) {return true;});
    EXPECT_EQ(result.decision, Decision::HeightJump);
    EXPECT_EQ(result.unknown_count, 2);
    EXPECT_FLOAT_EQ(result.observed_jump, 0.3f);
    samples[8].elevation = 0;
    EXPECT_EQ(evaluateNeighborhood(0, 0, Config{0.1, 0.3f, unknown_pass}, samples,
        [](double, double) {return true;}).decision, Decision::Outside);
    samples[8].elevation = 0.3f;
  }
  EXPECT_EQ(evaluateNeighborhood(std::numeric_limits<double>::quiet_NaN(), 0,
      Config{}, samples, [](double, double) {return true;}).decision, Decision::Invalid);
}
