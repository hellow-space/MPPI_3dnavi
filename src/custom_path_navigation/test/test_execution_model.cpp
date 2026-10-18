#include <gtest/gtest.h>

#include "custom_path_navigation/execution_model.hpp"

using custom_path_navigation::ExecutionDecision;
using custom_path_navigation::ExecutionModel;
using custom_path_navigation::ExecutionState;
using custom_path_navigation::TerminalResult;

TEST(ExecutionModel, ValidCandidateTransitionsIdleToReady)
{
  ExecutionModel model;
  EXPECT_EQ(model.state(), ExecutionState::IDLE);
  model.setCandidate(true);
  EXPECT_EQ(model.state(), ExecutionState::READY);
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::SendGoal);
  EXPECT_EQ(model.state(), ExecutionState::SENDING);
  EXPECT_EQ(model.generation(), 1U);
  model.goalAccepted(1);
  EXPECT_EQ(model.state(), ExecutionState::ACTIVE);
}

TEST(ExecutionModel, InvalidCandidateClearsExecutableCandidate)
{
  ExecutionModel model;
  model.setCandidate(true);
  model.setCandidate(false);
  EXPECT_EQ(model.state(), ExecutionState::FAILED);
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::Rejected);
  EXPECT_EQ(model.generation(), 0U);
}

TEST(ExecutionModel, ExecuteWithoutCandidateIsRejected)
{
  ExecutionModel model;
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::Rejected);
  EXPECT_EQ(model.generation(), 0U);
}

TEST(ExecutionModel, ReplacementWaitsForConfirmedCancellation)
{
  ExecutionModel model;
  model.setCandidate(true);
  model.requestExecute();
  model.goalAccepted(1);
  model.setCandidate(true);
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::CancelGoal);
  EXPECT_TRUE(model.hasPendingReplacement());
  EXPECT_EQ(model.generation(), 1U);
  EXPECT_EQ(model.cancelConfirmed(1), ExecutionDecision::SendGoal);
  EXPECT_EQ(model.generation(), 2U);
  EXPECT_EQ(model.state(), ExecutionState::SENDING);
  EXPECT_FALSE(model.hasPendingReplacement());
  model.goalAccepted(2);
  EXPECT_EQ(model.state(), ExecutionState::ACTIVE);
}

TEST(ExecutionModel, RepeatedExecuteWhileCancelPendingDoesNotDuplicateReplacement)
{
  ExecutionModel model;
  model.setCandidate(true);
  model.requestExecute();
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::Rejected);
  model.goalAccepted(1);
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::CancelGoal);
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::Rejected);
  EXPECT_EQ(model.generation(), 1U);
  EXPECT_EQ(model.cancelConfirmed(1), ExecutionDecision::SendGoal);
  EXPECT_EQ(model.cancelConfirmed(1), ExecutionDecision::Rejected);
  EXPECT_EQ(model.generation(), 2U);
}

TEST(ExecutionModel, LateOldGoalResultCannotOverwriteNewGeneration)
{
  ExecutionModel model;
  model.setCandidate(true);
  model.requestExecute();
  model.goalAccepted(1);
  model.requestExecute();
  model.cancelConfirmed(1);
  model.goalAccepted(2);
  model.goalFinished(1, TerminalResult::Failed);
  model.goalAccepted(1);
  EXPECT_EQ(model.state(), ExecutionState::ACTIVE);
  model.goalFinished(2, TerminalResult::Succeeded);
  EXPECT_EQ(model.state(), ExecutionState::SUCCEEDED);
}

TEST(ExecutionModel, RepeatedCancelIsIdempotent)
{
  ExecutionModel model;
  model.setCandidate(true);
  model.requestExecute();
  model.goalAccepted(1);
  EXPECT_EQ(model.requestCancel(), ExecutionDecision::CancelGoal);
  EXPECT_EQ(model.requestCancel(), ExecutionDecision::Rejected);
  EXPECT_EQ(model.cancelConfirmed(1), ExecutionDecision::Rejected);
  EXPECT_EQ(model.state(), ExecutionState::CANCELED);
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::Rejected);
}

TEST(ExecutionModel, CancelFailureRetainsOldGoalAndDropsReplacement)
{
  ExecutionModel model;
  model.setCandidate(true);
  model.requestExecute();
  model.goalAccepted(1);
  model.requestExecute();
  model.goalFinished(1, TerminalResult::CancelFailed);
  EXPECT_EQ(model.state(), ExecutionState::FAILED);
  EXPECT_FALSE(model.hasPendingReplacement());
  EXPECT_EQ(model.generation(), 1U);
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::CancelGoal);
}

TEST(ExecutionModel, InvalidCandidateDuringCancelDropsPendingReplacement)
{
  ExecutionModel model;
  model.setCandidate(true);
  model.requestExecute();
  model.goalAccepted(1);
  model.requestExecute();
  model.setCandidate(false);
  EXPECT_EQ(model.cancelConfirmed(1), ExecutionDecision::Rejected);
  EXPECT_EQ(model.generation(), 1U);
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::Rejected);
}

TEST(ExecutionModel, RejectedGoalAndFailedGoalBecomeFailed)
{
  ExecutionModel model;
  model.setCandidate(true);
  model.requestExecute();
  model.goalFinished(1, TerminalResult::Failed);
  EXPECT_EQ(model.state(), ExecutionState::FAILED);
  EXPECT_EQ(model.requestExecute(), ExecutionDecision::SendGoal);
  model.goalAccepted(2);
  model.goalFinished(2, TerminalResult::Canceled);
  EXPECT_EQ(model.state(), ExecutionState::CANCELED);
}
