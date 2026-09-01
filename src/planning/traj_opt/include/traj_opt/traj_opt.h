#pragma once
#include <mapping/mapping.h>
#include <ros/ros.h>

#include <memory>
#include <string>

#include "minco.hpp"

namespace traj_opt {

class TrajOpt {
 public:
  ros::NodeHandle nh_;
  // # pieces and # key points
  int N_, K_, dim_t_, dim_p_;
  // weight for time regularization term
  double rhoT_;
  // collision avoiding and dynamics paramters
  double vmax_, amax_;
  double rhoP_, rhoV_, rhoA_;
  double rhoTracking_, rhosVisibility_;
  double clearance_d_, tolerance_d_, theta_clearance_;
  // corridor
  std::vector<Eigen::MatrixXd> cfgVs_;
  std::vector<Eigen::MatrixXd> cfgHs_;
  // Minimum Jerk Optimizer
  minco::MinJerkOpt jerkOpt_;
  // weight for each vertex
  Eigen::VectorXd p_;
  // duration of each piece of the trajectory
  Eigen::VectorXd t_;
  double* x_;
  double sum_T_;

  std::vector<Eigen::Vector3d> tracking_ps_;
  std::vector<Eigen::Vector3d> tracking_visible_ps_;
  std::vector<double> tracking_thetas_;
  double tracking_dur_;
  double tracking_dist_;
  double tracking_dt_;

  std::shared_ptr<mapping::OccGridMap> mapPtr_;
  std::string visibility_model_;
  double rhoVisibilityRay_;
  int visibility_ray_samples_;
  double visibility_T_safe_;
  double visibility_occ_max_prob_;
  double visibility_unknown_prob_;
  double visibility_eps_;
  bool visibility_debug_;

  // polyH utils
  bool extractVs(const std::vector<Eigen::MatrixXd>& hPs,
                 std::vector<Eigen::MatrixXd>& vPs) const;

 public:
  TrajOpt(ros::NodeHandle& nh,
          const std::shared_ptr<mapping::OccGridMap>& map_ptr = nullptr);
  ~TrajOpt() {}

  void setBoundConds(const Eigen::MatrixXd& iniState, const Eigen::MatrixXd& finState);
  int optimize(const double& delta = 1e-4);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::Vector3d>& target_predcit,
                     const std::vector<Eigen::Vector3d>& visible_ps,
                     const std::vector<double>& thetas,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::Vector3d>& target_predcit,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);
  bool generate_traj(const Eigen::MatrixXd& iniState,
                     const Eigen::MatrixXd& finState,
                     const std::vector<Eigen::MatrixXd>& hPolys,
                     Trajectory& traj);

  void addTimeIntPenalty(double& cost);
  void addTimeCost(double& cost);
  bool grad_cost_p_corridor(const Eigen::Vector3d& p,
                            const Eigen::MatrixXd& hPoly,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool grad_cost_p_tracking(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& target_p,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool grad_cost_p_landing(const Eigen::Vector3d& p,
                           const Eigen::Vector3d& target_p,
                           Eigen::Vector3d& gradp,
                           double& costp);
  bool grad_cost_visibility(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& center,
                            const Eigen::Vector3d& vis_p,
                            const double& theta,
                            Eigen::Vector3d& gradp,
                            double& costp);
  bool evaluateRayOpticalDepth(const Eigen::Vector3d& p,
                               const Eigen::Vector3d& target_p,
                               double& tau,
                               Eigen::Vector3d& grad_tau) const;
  bool grad_cost_visibility_ray(const Eigen::Vector3d& p,
                                const Eigen::Vector3d& target_p,
                                Eigen::Vector3d& gradp,
                                double& costp) const;
  bool checkVisibilityRayGradient(const Eigen::Vector3d& p,
                                  const Eigen::Vector3d& target_p,
                                  const double finite_difference_step,
                                  Eigen::Vector3d& grad_analytic,
                                  Eigen::Vector3d& grad_finite_difference,
                                  double& relative_error) const;
  bool grad_cost_v(const Eigen::Vector3d& v,
                   Eigen::Vector3d& gradv,
                   double& costv);
  bool grad_cost_a(const Eigen::Vector3d& a,
                   Eigen::Vector3d& grada,
                   double& costa);
};

}  // namespace traj_opt
