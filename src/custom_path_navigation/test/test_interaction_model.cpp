#include <gtest/gtest.h>

#include "custom_path_navigation/interaction_model.hpp"

using custom_path_navigation::InteractionModel;

namespace
{
void draw(InteractionModel & model, double x)
{
  model.beginStroke("map");
  model.append({x, 0.0});
  model.append({x + 1.0, 2.0});
  ASSERT_TRUE(model.finishStroke());
}
}  // namespace

TEST(InteractionModel, MouseReleaseCreatesReadyCandidateWithoutExecuteCommand)
{
  InteractionModel model;
  model.beginStroke("map");
  model.append({0.0, 1.0});
  model.append({2.0, 3.0});
  EXPECT_FALSE(model.currentPath());
  EXPECT_TRUE(model.drawing());
  EXPECT_TRUE(model.finishStroke());
  ASSERT_TRUE(model.currentPath());
  EXPECT_EQ(model.currentPath()->frame, "map");
  ASSERT_EQ(model.currentPath()->points.size(), 2u);
  EXPECT_DOUBLE_EQ(model.currentPath()->points.back().x, 2.0);
  EXPECT_DOUBLE_EQ(model.currentPath()->points.back().y, 3.0);
  EXPECT_FALSE(model.drawing());
  EXPECT_FALSE(model.previousPath());
}

TEST(InteractionModel, NewStrokeReplacesCandidateAndKeepsOneUndoLevel)
{
  InteractionModel model;
  draw(model, 0.0);
  draw(model, 3.0);
  draw(model, 6.0);
  ASSERT_TRUE(model.currentPath());
  ASSERT_TRUE(model.previousPath());
  EXPECT_DOUBLE_EQ(model.currentPath()->points.front().x, 6.0);
  EXPECT_DOUBLE_EQ(model.previousPath()->points.front().x, 3.0);
}

TEST(InteractionModel, CtrlZRestoresPreviousCandidateOnlyOnce)
{
  InteractionModel model;
  EXPECT_FALSE(model.undo());
  draw(model, 0.0);
  draw(model, 3.0);
  EXPECT_TRUE(model.undo());
  ASSERT_TRUE(model.currentPath());
  EXPECT_DOUBLE_EQ(model.currentPath()->points.front().x, 0.0);
  EXPECT_FALSE(model.previousPath());
  EXPECT_FALSE(model.undo());
  EXPECT_DOUBLE_EQ(model.currentPath()->points.front().x, 0.0);
}

TEST(InteractionModel, EscapeDuringDrawingDiscardsOnlyCurrentStroke)
{
  InteractionModel model;
  draw(model, 0.0);
  draw(model, 3.0);
  model.beginStroke("map");
  model.append({6.0, 0.0});
  model.cancelStroke();
  EXPECT_FALSE(model.drawing());
  EXPECT_FALSE(model.finishStroke());
  ASSERT_TRUE(model.currentPath());
  ASSERT_TRUE(model.previousPath());
  EXPECT_DOUBLE_EQ(model.currentPath()->points.front().x, 3.0);
  EXPECT_DOUBLE_EQ(model.previousPath()->points.front().x, 0.0);
}

TEST(InteractionModel, EscapeWhenReadyClearsCandidate)
{
  InteractionModel model;
  draw(model, 0.0);
  draw(model, 3.0);
  model.clear();
  EXPECT_FALSE(model.currentPath());
  EXPECT_FALSE(model.previousPath());
  EXPECT_FALSE(model.undo());
  EXPECT_FALSE(model.drawing());
}

TEST(InteractionModel, FixedFrameChangeDuringStrokeDiscardsMixedFrameStroke)
{
  InteractionModel model;
  draw(model, 0.0);
  model.beginStroke("map");
  model.append({3.0, 0.0});
  model.frameChanged("map");
  EXPECT_TRUE(model.drawing());
  model.frameChanged("odom");
  EXPECT_FALSE(model.drawing());
  model.append({4.0, 0.0});
  EXPECT_FALSE(model.finishStroke());
  ASSERT_TRUE(model.currentPath());
  EXPECT_EQ(model.currentPath()->frame, "map");
  EXPECT_DOUBLE_EQ(model.currentPath()->points.front().x, 0.0);
  model.beginStroke("odom");
  model.append({5.0, 0.0});
  model.append({6.0, 0.0});
  ASSERT_TRUE(model.finishStroke());
  EXPECT_EQ(model.currentPath()->frame, "odom");
  EXPECT_EQ(model.previousPath()->frame, "map");
}

TEST(InteractionModel, EmptyOrOnePointStrokeDoesNotReplaceValidHistory)
{
  InteractionModel model;
  draw(model, 0.0);
  draw(model, 3.0);
  model.beginStroke("map");
  EXPECT_FALSE(model.finishStroke());
  model.beginStroke("map");
  model.append({9.0, 0.0});
  EXPECT_FALSE(model.finishStroke());
  ASSERT_TRUE(model.currentPath());
  ASSERT_TRUE(model.previousPath());
  EXPECT_DOUBLE_EQ(model.currentPath()->points.front().x, 3.0);
  EXPECT_DOUBLE_EQ(model.previousPath()->points.front().x, 0.0);
}
