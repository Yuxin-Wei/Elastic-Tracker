#include <gtest/gtest.h>
#include <ros/ros.h>
#include <traj_opt/traj_opt.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string formatMetric(const double value) {
  std::ostringstream stream;
  stream << std::scientific << std::setprecision(9) << value;
  return stream.str();
}

class VisibilityRayTest : public ::testing::Test {
 protected:
  void SetUp() override {
    map_ = std::make_shared<mapping::OccGridMap>();
    map_->setup(1.0, Eigen::Vector3d::Constant(16.0), 10.0, true);
    for (int x = map_->offset_x; x < map_->offset_x + map_->size_x; ++x) {
      for (int y = map_->offset_y; y < map_->offset_y + map_->size_y; ++y) {
        for (int z = map_->offset_z; z < map_->offset_z + map_->size_z; ++z) {
          map_->setFree(Eigen::Vector3i(x, y, z));
        }
      }
    }

    ros::NodeHandle nh("~");
    traj_opt_.reset(new traj_opt::TrajOpt(nh, map_));
    traj_opt_->visibility_model_ = "ray";
    traj_opt_->rhoVisibilityRay_ = 10000.0;
    traj_opt_->visibility_ray_samples_ = 20;
    traj_opt_->visibility_T_safe_ = 0.80;
    traj_opt_->visibility_occ_max_prob_ = 0.95;
    traj_opt_->visibility_unknown_prob_ = 0.0;
    traj_opt_->visibility_eps_ = 1e-6;
    traj_opt_->visibility_debug_ = false;
  }

  void occupyOriginVoxel() {
    map_->setOcc(map_->idx2pos(Eigen::Vector3i::Zero()));
  }

  void expectGradientMatches(const Eigen::Vector3d& p,
                             const Eigen::Vector3d& target,
                             const double tolerance = 1e-3,
                             double* measured_relative_error = nullptr) {
    Eigen::Vector3d grad_analytic, grad_finite_difference;
    double relative_error;
    ASSERT_TRUE(traj_opt_->checkVisibilityRayGradient(
        p, target, 1e-5, grad_analytic, grad_finite_difference,
        relative_error));
    if (measured_relative_error != nullptr) {
      *measured_relative_error = relative_error;
    }
    EXPECT_LT(relative_error, tolerance);
  }

  void expectOpticalDepthGradientMatches(const Eigen::Vector3d& p,
                                         const Eigen::Vector3d& target,
                                         const double tolerance = 1e-3,
                                         double* measured_relative_error = nullptr) {
    double tau;
    Eigen::Vector3d grad_analytic;
    ASSERT_TRUE(traj_opt_->evaluateRayOpticalDepth(
        p, target, tau, grad_analytic));

    Eigen::Vector3d grad_finite_difference;
    const double step = 1e-5;
    for (int axis = 0; axis < 3; ++axis) {
      Eigen::Vector3d p_plus = p;
      Eigen::Vector3d p_minus = p;
      p_plus(axis) += step;
      p_minus(axis) -= step;

      double tau_plus, tau_minus;
      Eigen::Vector3d unused_grad;
      ASSERT_TRUE(traj_opt_->evaluateRayOpticalDepth(
          p_plus, target, tau_plus, unused_grad));
      ASSERT_TRUE(traj_opt_->evaluateRayOpticalDepth(
          p_minus, target, tau_minus, unused_grad));
      grad_finite_difference(axis) =
          (tau_plus - tau_minus) / (2.0 * step);
    }

    const double relative_error =
        (grad_analytic - grad_finite_difference).norm() /
        std::max(1.0, grad_finite_difference.norm());
    if (measured_relative_error != nullptr) {
      *measured_relative_error = relative_error;
    }
    EXPECT_LT(relative_error, tolerance);
  }

  std::shared_ptr<mapping::OccGridMap> map_;
  std::unique_ptr<traj_opt::TrajOpt> traj_opt_;
};

TEST_F(VisibilityRayTest, OccupancyProxyHasAnalyticalGradient) {
  occupyOriginVoxel();
  const Eigen::Vector3d p(0.75, 0.65, 0.60);

  double occ;
  Eigen::Vector3d grad;
  ASSERT_TRUE(map_->queryOccupancyProxy(p, 0.0, occ, grad));

  Eigen::Vector3d grad_finite_difference;
  const double step = 1e-6;
  for (int axis = 0; axis < 3; ++axis) {
    Eigen::Vector3d plus = p;
    Eigen::Vector3d minus = p;
    plus(axis) += step;
    minus(axis) -= step;
    double occ_plus, occ_minus;
    Eigen::Vector3d unused;
    ASSERT_TRUE(map_->queryOccupancyProxy(plus, 0.0, occ_plus, unused));
    ASSERT_TRUE(map_->queryOccupancyProxy(minus, 0.0, occ_minus, unused));
    grad_finite_difference(axis) = (occ_plus - occ_minus) / (2.0 * step);
  }
  EXPECT_LT((grad - grad_finite_difference).norm(), 1e-8);
}

TEST_F(VisibilityRayTest, UnknownSamplesUseConfiguredProbability) {
  mapping::OccGridMap unknown_map;
  unknown_map.setup(1.0, Eigen::Vector3d::Constant(16.0), 10.0, true);

  double occ;
  Eigen::Vector3d grad;
  ASSERT_TRUE(unknown_map.queryOccupancyProxy(Eigen::Vector3d(0.73, 0.61, 0.82),
                                             0.35, occ, grad));
  EXPECT_NEAR(occ, 0.35, 1e-12);
  EXPECT_LT(grad.norm(), 1e-12);

  ASSERT_TRUE(unknown_map.queryOccupancyProxy(Eigen::Vector3d::Constant(100.0),
                                             0.35, occ, grad));
  EXPECT_NEAR(occ, 0.35, 1e-12);
  EXPECT_LT(grad.norm(), 1e-12);
}

TEST_F(VisibilityRayTest, FreeLineOfSightHasUnitTransmittance) {
  const Eigen::Vector3d target(-4.37, -0.23, 0.17);
  const Eigen::Vector3d p(4.19, 0.31, 0.27);

  double tau;
  Eigen::Vector3d grad_tau;
  ASSERT_TRUE(traj_opt_->evaluateRayOpticalDepth(p, target, tau, grad_tau));
  EXPECT_NEAR(std::exp(-tau), 1.0, 1e-4);

  Eigen::Vector3d grad_cost;
  double cost;
  EXPECT_FALSE(traj_opt_->grad_cost_visibility_ray(p, target, grad_cost, cost));
  EXPECT_DOUBLE_EQ(cost, 0.0);
  EXPECT_TRUE(grad_cost.isZero());
  expectGradientMatches(p, target);
}

TEST_F(VisibilityRayTest, VeryShortRayIsRobust) {
  const Eigen::Vector3d target(0.73, 0.61, 0.82);
  double tau;
  Eigen::Vector3d grad_tau;
  ASSERT_TRUE(traj_opt_->evaluateRayOpticalDepth(target, target, tau, grad_tau));
  EXPECT_DOUBLE_EQ(tau, 0.0);
  EXPECT_TRUE(grad_tau.isZero());

  Eigen::Vector3d grad_cost;
  double cost;
  EXPECT_FALSE(traj_opt_->grad_cost_visibility_ray(target, target, grad_cost, cost));
  EXPECT_DOUBLE_EQ(cost, 0.0);
  EXPECT_TRUE(grad_cost.isZero());
}

TEST_F(VisibilityRayTest, BlockedLineOfSightReducesTransmittance) {
  occupyOriginVoxel();
  const Eigen::Vector3d target(-4.20, 0.15, 0.12);
  const Eigen::Vector3d p(4.35, 0.19, 0.18);

  double tau;
  Eigen::Vector3d grad_tau;
  ASSERT_TRUE(traj_opt_->evaluateRayOpticalDepth(p, target, tau, grad_tau));
  EXPECT_LT(std::exp(-tau), 0.70);

  Eigen::Vector3d grad_cost;
  double cost;
  EXPECT_TRUE(traj_opt_->grad_cost_visibility_ray(p, target, grad_cost, cost));
  EXPECT_GT(cost, 0.0);
  expectGradientMatches(p, target);
}

TEST_F(VisibilityRayTest, ObstacleBoundaryProducesLateralGradient) {
  occupyOriginVoxel();
  const Eigen::Vector3d target(-4.10, 0.20, 0.22);
  const Eigen::Vector3d p(4.40, 1.05, 0.35);

  double tau;
  Eigen::Vector3d grad_tau;
  ASSERT_TRUE(traj_opt_->evaluateRayOpticalDepth(p, target, tau, grad_tau));
  EXPECT_GT(std::abs(grad_tau.y()), 1e-2);
  expectGradientMatches(p, target);
}

TEST_F(VisibilityRayTest, OcclusionApproachHasConsistentClearanceGradient) {
  occupyOriginVoxel();
  const Eigen::Vector3d target(-4.5, 0.5, 0.43);
  const std::vector<double> offsets{1.8, 1.5, 1.2, 0.9, 0.6, 0.3};
  int active_transition_samples = 0;
  double max_tau_relative_error = 0.0;
  double max_cost_relative_error = 0.0;
  double min_tau_clearance_projection = std::numeric_limits<double>::infinity();
  double min_cost_clearance_projection = std::numeric_limits<double>::infinity();
  double min_far_transmittance = std::numeric_limits<double>::infinity();
  double max_near_transmittance = 0.0;

  for (const double side : {-1.0, 1.0}) {
    double previous_transmittance = std::numeric_limits<double>::infinity();
    for (const double offset : offsets) {
      SCOPED_TRACE("side=" + std::to_string(side) +
                   ", offset=" + std::to_string(offset));
      const Eigen::Vector3d p(5.5, 0.5 + side * offset, 0.43);

      double tau;
      Eigen::Vector3d grad_tau;
      ASSERT_TRUE(traj_opt_->evaluateRayOpticalDepth(
          p, target, tau, grad_tau));
      const double transmittance = std::exp(-tau);
      EXPECT_LT(transmittance, previous_transmittance);
      previous_transmittance = transmittance;
      if (offset == offsets.front()) {
        min_far_transmittance = std::min(min_far_transmittance, transmittance);
      }
      if (offset == offsets.back()) {
        max_near_transmittance = std::max(max_near_transmittance, transmittance);
      }

      double tau_relative_error = std::numeric_limits<double>::infinity();
      expectOpticalDepthGradientMatches(
          p, target, 1e-3, &tau_relative_error);
      max_tau_relative_error =
          std::max(max_tau_relative_error, tau_relative_error);

      if (offset <= 1.2) {
        Eigen::Vector3d grad_cost;
        double cost;
        ASSERT_TRUE(traj_opt_->grad_cost_visibility_ray(
            p, target, grad_cost, cost));
        EXPECT_LT(transmittance, traj_opt_->visibility_T_safe_);
        EXPECT_GT(cost, 0.0);

        const Eigen::Vector3d ray_direction = (p - target).normalized();
        const Eigen::Vector3d outward(0.0, side, 0.0);
        const Eigen::Vector3d outward_lateral =
            outward - outward.dot(ray_direction) * ray_direction;
        const Eigen::Vector3d grad_tau_lateral =
            grad_tau - grad_tau.dot(ray_direction) * ray_direction;
        const Eigen::Vector3d grad_cost_lateral =
            grad_cost - grad_cost.dot(ray_direction) * ray_direction;

        ASSERT_GT(outward_lateral.norm(), 1e-6);
        EXPECT_GT(grad_tau_lateral.norm(), 1e-3);
        const double tau_clearance_projection =
            (-grad_tau_lateral).dot(outward_lateral.normalized());
        const double cost_clearance_projection =
            (-grad_cost_lateral).dot(outward_lateral.normalized());
        EXPECT_GT(tau_clearance_projection, 1e-3);
        EXPECT_GT(cost_clearance_projection, 1e-3);
        min_tau_clearance_projection =
            std::min(min_tau_clearance_projection, tau_clearance_projection);
        min_cost_clearance_projection =
            std::min(min_cost_clearance_projection, cost_clearance_projection);

        double cost_relative_error = std::numeric_limits<double>::infinity();
        expectGradientMatches(p, target, 1e-3, &cost_relative_error);
        max_cost_relative_error =
            std::max(max_cost_relative_error, cost_relative_error);
        ++active_transition_samples;
      }
    }
  }

  EXPECT_EQ(active_transition_samples, 8);
  RecordProperty("max_tau_relative_error",
                 formatMetric(max_tau_relative_error));
  RecordProperty("max_cost_relative_error",
                 formatMetric(max_cost_relative_error));
  RecordProperty("min_tau_clearance_projection",
                 formatMetric(min_tau_clearance_projection));
  RecordProperty("min_cost_clearance_projection",
                 formatMetric(min_cost_clearance_projection));
  RecordProperty("min_far_transmittance",
                 formatMetric(min_far_transmittance));
  RecordProperty("max_near_transmittance",
                 formatMetric(max_near_transmittance));
}

TEST_F(VisibilityRayTest, RandomRaysNearObstacleBoundaryMatchFiniteDifferences) {
  occupyOriginVoxel();
  const Eigen::Vector3d target(-4.10, 0.20, 0.22);
  std::mt19937 generator(7);
  std::uniform_real_distribution<double> x_distribution(4.0, 4.8);
  std::uniform_real_distribution<double> y_distribution(0.9, 1.25);
  std::uniform_real_distribution<double> z_distribution(0.25, 0.75);

  for (int sample = 0; sample < 12; ++sample) {
    const Eigen::Vector3d p(x_distribution(generator),
                            y_distribution(generator),
                            z_distribution(generator));
    expectGradientMatches(p, target, 1e-2);
  }
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_visibility_ray",
            ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
