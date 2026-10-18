#ifndef CUSTOM_PATH_NAVIGATION__EXECUTION_MODEL_HPP_
#define CUSTOM_PATH_NAVIGATION__EXECUTION_MODEL_HPP_

#include <cstdint>

namespace custom_path_navigation
{
enum class ExecutionState {IDLE, READY, SENDING, ACTIVE, SUCCEEDED, FAILED, CANCELED};
enum class ExecutionDecision {Rejected, SendGoal, CancelGoal};
// CancelFailed reports failure without pretending the old action has terminated.
enum class TerminalResult {Succeeded, Failed, Canceled, CancelFailed};

class ExecutionModel
{
public:
  void setCandidate(bool valid);
  ExecutionDecision requestExecute();
  void goalAccepted(uint64_t generation);
  ExecutionDecision cancelConfirmed(uint64_t generation);
  void goalFinished(uint64_t generation, TerminalResult result);
  ExecutionDecision requestCancel();
  ExecutionState state() const {return state_;}
  uint64_t generation() const {return generation_;}
  bool hasPendingReplacement() const {return pending_replacement_;}

private:
  ExecutionDecision beginGoal();
  ExecutionState state_{ExecutionState::IDLE};
  uint64_t generation_{0};
  bool candidate_valid_{false};
  bool goal_outstanding_{false};
  bool cancel_pending_{false};
  bool pending_replacement_{false};
};
const char * stateName(ExecutionState state);
}  // namespace custom_path_navigation
#endif
