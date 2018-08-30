#include <eigen-checks/gtest.h>
#include <mav_trajectory_generation/trajectory_sampling.h>

#include "loco_planner/loco.h"

using namespace mav_trajectory_generation;

namespace loco_planner {
class LocoTest : public ::testing::Test {
 protected:
  LocoTest()
      : num_segments_(3),
        total_time_(10.0),
        resolution_(0.1),
        max_distance_(2.0),
        obstacle_radius_(1.0),
        loco_(2) {}

  virtual void SetUp() {
    // Default values, easily overwritten.
    obstacle_center_ << 2.0, 2.0;
    start_ << 0.0, 0.5;
    goal_ << 3.5, 3.0;
    loco_.setEpsilon(0.5);
    loco_.setRobotRadius(0.5);
    loco_.setVerbose(false);
    setExactDistanceFunction();
  }

  // Set up different distance callbacks in LOCO.
  // Exact Distance Function is default.
  void setExactDistanceFunction() {
    loco_.setDistanceFunction(std::bind(&LocoTest::getExactDistanceAndGradient,
                                        this, std::placeholders::_1, nullptr));
  }
  void setExactDistanceAndGradientFunction() {
    loco_.setDistanceAndGradientFunction(
        std::bind(&LocoTest::getExactDistanceAndGradient, this,
                  std::placeholders::_1, std::placeholders::_2));
  }
  void setDiscretizedDistanceFunction() {}
  void setDiscretizedDistanceAndGradientFunction() {}

  // Our map is computed analytically. It's basically an infinite plane
  // with a max distance of max_distance_, with a sphere of radius 1.0
  // at 2.0, 2.0.
  // Thes function allows querying the distance.
  double getExactDistanceAndGradient(const Eigen::VectorXd& pos,
                                     Eigen::VectorXd* gradient) const {
    Eigen::Vector2d vector_from_center = pos.head<2>() - obstacle_center_;
    double distance = (vector_from_center).norm() - obstacle_radius_;
    if (gradient != nullptr) {
      *gradient = (vector_from_center) / (vector_from_center).norm();
    }
    return distance;
  }

  // Check trajectory for collisions.
  bool inCollision(const Trajectory& trajectory) const {
    double robot_radius = loco_.getRobotRadius();

    std::vector<Eigen::VectorXd> result;
    std::vector<double> sampling_times;
    trajectory.evaluateRange(trajectory.getMinTime(), trajectory.getMaxTime(),
                             0.01, 0, &result, &sampling_times);
    for (const Eigen::VectorXd& point : result) {
      if (getExactDistanceAndGradient(point, nullptr) < robot_radius) {
        return true;
      }
    }
    return false;
  }

  // TODO(helenol): add voxel-discretized versions down to resolution.

  Eigen::Vector2d start_;
  Eigen::Vector2d goal_;
  int num_segments_;
  double total_time_;
  double resolution_;
  double max_distance_;

  // Map obstacle parameters:
  double obstacle_radius_;
  Eigen::Vector2d obstacle_center_;

  Loco<10> loco_;
};

// This tests makes sure that the initial solution is already a local minimum.
TEST_F(LocoTest, VerifySmoothnessCostAndGrad) {
  // Let's start easy.
  loco_.setupFromPositions(start_, goal_, num_segments_, total_time_);

  std::vector<Eigen::VectorXd> gradients;
  double cost = loco_.computeDerivativeCostAndGradient(&gradients);

  for (const Eigen::VectorXd& grad : gradients) {
    // Verify that these are around ~1e-8.
    EXPECT_TRUE(EIGEN_MATRIX_ZERO(grad, 1e-6));
  }

  // This is dependent on number of segments...
  EXPECT_GT(0.5, cost);
  // loco_.printMatlabSampledTrajectory(0.1);
}

TEST_F(LocoTest, VerifyPotentialCosts) {
  setExactDistanceAndGradientFunction();
  loco_.setupFromPositions(start_, goal_, num_segments_, total_time_);

  constexpr double kGradientTolerance = 1e-2;

  // This is near the edge of the obstacle.
  Eigen::Vector2d test_pos(1.0, 1.0);
  Eigen::VectorXd grad_a, gradient;
  double cost = loco_.computePotentialCostAndGradient(test_pos, &grad_a);
  std::cout << "Cost: " << cost << std::endl;
  EXPECT_GT(cost, 0.3);
  // I guess gradient should actually be 0 here. But it's not. Because my math
  // sucks.

  double h = 0.1;
  Eigen::Vector2d grad_n(0, 0);
  Eigen::Vector2d increment(0, 0);
  for (int k = 0; k < 2; ++k) {
    increment.setZero();
    increment(k) = h;
    double cost_p =
        loco_.computePotentialCostAndGradient(test_pos + increment, &gradient);
    double cost_n =
        loco_.computePotentialCostAndGradient(test_pos - increment, &gradient);

    grad_n(k) = (cost_p - cost_n) / (2.0 * h);
  }
  std::cout << "Position: " << test_pos.transpose() << " cost: " << cost
            << " h: " << h << "\ngrad_a: " << grad_a.transpose()
            << "\ngrad_n: " << grad_n.transpose() << std::endl;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(grad_a, grad_n, kGradientTolerance));

  h = 0.02;
  for (int k = 0; k < 2; ++k) {
    increment.setZero();
    increment(k) = h;
    double cost_p =
        loco_.computePotentialCostAndGradient(test_pos + increment, &gradient);
    double cost_n =
        loco_.computePotentialCostAndGradient(test_pos - increment, &gradient);

    grad_n(k) = (cost_p - cost_n) / (2.0 * h);
  }
  std::cout << "Position: " << test_pos.transpose() << " cost: " << cost
            << " h: " << h << "\ngrad_a: " << grad_a.transpose()
            << "\ngrad_n: " << grad_n.transpose() << std::endl;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(grad_a, grad_n, kGradientTolerance));

  // Try another position.
  test_pos << 1.0, 1.5;
  cost = loco_.computePotentialCostAndGradient(test_pos, &grad_a);
  h = 0.1;
  for (int k = 0; k < 2; ++k) {
    increment.setZero();
    increment(k) = h;
    double cost_p =
        loco_.computePotentialCostAndGradient(test_pos + increment, &gradient);
    double cost_n =
        loco_.computePotentialCostAndGradient(test_pos - increment, &gradient);

    grad_n(k) = (cost_p - cost_n) / (2.0 * h);
  }
  std::cout << "Position: " << test_pos.transpose() << " cost: " << cost
            << " h: " << h << "\ngrad_a: " << grad_a.transpose()
            << "\ngrad_n: " << grad_n.transpose() << std::endl;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(grad_a, grad_n, kGradientTolerance));

  h = 0.02;
  for (int k = 0; k < 2; ++k) {
    increment.setZero();
    increment(k) = h;
    double cost_p =
        loco_.computePotentialCostAndGradient(test_pos + increment, &gradient);
    double cost_n =
        loco_.computePotentialCostAndGradient(test_pos - increment, &gradient);

    grad_n(k) = (cost_p - cost_n) / (2.0 * h);
  }
  std::cout << "Position: " << test_pos.transpose() << " cost: " << cost
            << " h: " << h << "\ngrad_a: " << grad_a.transpose()
            << "\ngrad_n: " << grad_n.transpose() << std::endl;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(grad_a, grad_n, kGradientTolerance));

  h = 0.05;
  for (int k = 0; k < 2; ++k) {
    increment.setZero();
    increment(k) = h;
    double cost_p =
        loco_.computePotentialCostAndGradient(test_pos + increment, &gradient);
    double cost_n =
        loco_.computePotentialCostAndGradient(test_pos - increment, &gradient);

    grad_n(k) = (cost_p - cost_n) / (2.0 * h);
  }
  std::cout << "Position: " << test_pos.transpose() << " cost: " << cost
            << " h: " << h << "\ngrad_a: " << grad_a.transpose()
            << "\ngrad_n: " << grad_n.transpose() << std::endl;
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(grad_a, grad_n, kGradientTolerance));
}

TEST_F(LocoTest, TestCeres) {
  setExactDistanceAndGradientFunction();
  loco_.setupFromPositions(start_, goal_, num_segments_, total_time_);

  // Check cost.
  std::vector<Eigen::VectorXd> gradients;
  std::vector<Eigen::VectorXd> gradients_c;
  std::vector<Eigen::VectorXd> gradients_d;

  double cost_c = loco_.computeCollisionCostAndGradient(&gradients_c);
  double cost_d = loco_.computeDerivativeCostAndGradient(&gradients_d);
  double cost = loco_.computeTotalCostAndGradients(&gradients);
  std::cout << "Num free: " << loco_.getNumParams() << std::endl;

  std::cout << "Cost c: " << cost_c << " Cost d: " << cost_d
            << " Total cost: " << cost << std::endl;

  mav_trajectory_generation::Trajectory trajectory;
  loco_.getTrajectory(&trajectory);
  EXPECT_TRUE(inCollision(trajectory));

  loco_.solveProblem();

  double final_cost = loco_.computeTotalCostAndGradients(&gradients);

  std::cout << "Initial cost: " << cost << " Final cost: " << final_cost
            << std::endl;
  EXPECT_LT(final_cost, cost);

  loco_.getTrajectory(&trajectory);
  EXPECT_FALSE(inCollision(trajectory));
  // loco_.printMatlabSampledTrajectory(0.1);
}


TEST_F(LocoTest, TestTrajectoryInitialization) {
  setExactDistanceAndGradientFunction();
  loco_.setupFromPositions(start_, goal_, num_segments_, total_time_);
  //loco_.setVerbose(true);

  mav_trajectory_generation::Trajectory trajectory;
  Eigen::VectorXd params_solve1, params_before, params_after;
  int num_params = loco_.getNumParams();

  loco_.solveProblem();
  double first_solve_cost = loco_.computeTotalCostAndGradients(nullptr);
  loco_.getTrajectory(&trajectory);
  EXPECT_FALSE(inCollision(trajectory));
  loco_.getParameterVector(&params_solve1);
  EXPECT_EQ(num_params, loco_.getNumParams());

  loco_.setupFromTrajectory(trajectory);
  double second_before_cost = loco_.computeTotalCostAndGradients(nullptr);
  EXPECT_NEAR(first_solve_cost, second_before_cost, 1e-4);
  loco_.getParameterVector(&params_before);
  EXPECT_EQ(num_params, loco_.getNumParams());

  loco_.solveProblem();
  double second_after_cost = loco_.computeTotalCostAndGradients(nullptr);
  EXPECT_NEAR(first_solve_cost, second_after_cost, 1e-4);
  loco_.getParameterVector(&params_after);
  EXPECT_EQ(num_params, loco_.getNumParams());

  loco_.getTrajectory(&trajectory);
  EXPECT_FALSE(inCollision(trajectory));

  EXPECT_TRUE(EIGEN_MATRIX_NEAR(params_solve1, params_before, 1e-4));
  EXPECT_TRUE(EIGEN_MATRIX_NEAR(params_before, params_after, 1e-4));
}


}  // namespace loco_planner

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  google::InitGoogleLogging(argv[0]);

  int result = RUN_ALL_TESTS();
  timing::Timing::Print(std::cout);

  return result;
}
