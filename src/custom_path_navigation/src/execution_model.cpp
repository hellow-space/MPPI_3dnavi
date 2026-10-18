#include "custom_path_navigation/execution_model.hpp"

namespace custom_path_navigation
{
void ExecutionModel::setCandidate(bool valid)
{
  candidate_valid_ = valid;
  if (!valid) {
    pending_replacement_ = false;
  }
  if (!goal_outstanding_) {
    state_ = valid ? ExecutionState::READY : ExecutionState::FAILED;
  }
}

ExecutionDecision ExecutionModel::beginGoal()
{
  ++generation_;
  goal_outstanding_ = true;
  cancel_pending_ = false;
  pending_replacement_ = false;
  state_ = ExecutionState::SENDING;
  return ExecutionDecision::SendGoal;
}

ExecutionDecision ExecutionModel::requestExecute()
{
  if (!candidate_valid_ || cancel_pending_ || state_ == ExecutionState::SENDING) {
    return ExecutionDecision::Rejected;
  }
  if (goal_outstanding_) {
    cancel_pending_ = true;
    pending_replacement_ = true;
    return ExecutionDecision::CancelGoal;
  }
  return beginGoal();
}

void ExecutionModel::goalAccepted(uint64_t generation)
{
  if (generation == generation_ && goal_outstanding_ && state_ == ExecutionState::SENDING) {
    state_ = ExecutionState::ACTIVE;
  }
}

ExecutionDecision ExecutionModel::cancelConfirmed(uint64_t generation)
{
  if (generation != generation_ || !goal_outstanding_ || !cancel_pending_) {
    return ExecutionDecision::Rejected;
  }
  goal_outstanding_ = false;
  cancel_pending_ = false;
  const bool replace = pending_replacement_ && candidate_valid_;
  pending_replacement_ = false;
  state_ = ExecutionState::CANCELED;
  return replace ? beginGoal() : ExecutionDecision::Rejected;
}

void ExecutionModel::goalFinished(uint64_t generation, TerminalResult result)
{
  if (generation != generation_ || !goal_outstanding_) {
    return;
  }
  cancel_pending_ = false;
  pending_replacement_ = false;
  if (result != TerminalResult::CancelFailed) {
    goal_outstanding_ = false;
  }
  state_ = result == TerminalResult::Succeeded ? ExecutionState::SUCCEEDED :
    result == TerminalResult::Canceled ? ExecutionState::CANCELED : ExecutionState::FAILED;
}

ExecutionDecision ExecutionModel::requestCancel()
{
  candidate_valid_ = false;
  pending_replacement_ = false;
  if (cancel_pending_) {
    return ExecutionDecision::Rejected;
  }
  if (goal_outstanding_) {
    cancel_pending_ = true;
    return ExecutionDecision::CancelGoal;
  }
  state_ = ExecutionState::CANCELED;
  return ExecutionDecision::Rejected;
}

const char * stateName(ExecutionState state)
{
  switch (state) {
    case ExecutionState::IDLE: return "IDLE";
    case ExecutionState::READY: return "READY";
    case ExecutionState::SENDING: return "SENDING";
    case ExecutionState::ACTIVE: return "ACTIVE";
    case ExecutionState::SUCCEEDED: return "SUCCEEDED";
    case ExecutionState::FAILED: return "FAILED";
    case ExecutionState::CANCELED: return "CANCELED";
  }
  return "FAILED";
}
}  // namespace custom_path_navigation
