#include "custom_path_navigation/interaction_model.hpp"

#include <utility>

namespace custom_path_navigation
{
void InteractionModel::beginStroke(std::string frame)
{
  stroke_ = CandidatePath{std::move(frame), {}};
}

void InteractionModel::append(Point2D point)
{
  if (stroke_) {
    stroke_->points.push_back(point);
  }
}

bool InteractionModel::finishStroke()
{
  if (!stroke_ || stroke_->points.size() < 2) {
    cancelStroke();
    return false;
  }
  previous_ = std::move(current_);
  current_ = std::move(stroke_);
  stroke_.reset();
  return true;
}

void InteractionModel::cancelStroke()
{
  stroke_.reset();
}

bool InteractionModel::undo()
{
  if (!previous_) {
    return false;
  }
  cancelStroke();
  current_ = std::move(previous_);
  previous_.reset();
  return true;
}

void InteractionModel::clear()
{
  cancelStroke();
  current_.reset();
  previous_.reset();
}

void InteractionModel::frameChanged(std::string frame)
{
  if (stroke_ && stroke_->frame != frame) {
    cancelStroke();
  }
}
}  // namespace custom_path_navigation
