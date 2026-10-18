#ifndef CUSTOM_PATH_NAVIGATION__INTERACTION_MODEL_HPP_
#define CUSTOM_PATH_NAVIGATION__INTERACTION_MODEL_HPP_

#include <optional>
#include <string>
#include <vector>

namespace custom_path_navigation
{
struct Point2D
{
  double x;
  double y;
};

struct CandidatePath
{
  std::string frame;
  std::vector<Point2D> points;
};

/// Local candidate history only: finishing a stroke never requests execution.
class InteractionModel
{
public:
  void beginStroke(std::string frame);
  void append(Point2D point);
  bool finishStroke();
  void cancelStroke();
  bool undo();
  void clear();
  void frameChanged(std::string frame);
  const std::optional<CandidatePath> & currentPath() const {return current_;}
  const std::optional<CandidatePath> & previousPath() const {return previous_;}
  bool drawing() const {return stroke_.has_value();}

private:
  std::optional<CandidatePath> current_;
  std::optional<CandidatePath> previous_;
  std::optional<CandidatePath> stroke_;
};
}  // namespace custom_path_navigation
#endif
