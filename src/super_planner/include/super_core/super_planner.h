/**
* This file is part of SUPER
*
* Copyright 2025 Yunfan REN, MaRS Lab, University of Hong Kong, <mars.hku.hk>
* Developed by Yunfan REN <renyf at connect dot hku dot hk>
* for more information see <https://github.com/hku-mars/SUPER>.
* If you use this code, please cite the respective publications as
* listed on the above website.
*
* SUPER is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* SUPER is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with SUPER. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <iostream>
#include <fstream>
#include <cstdint>
#include <mutex>
#include "Eigen/Eigen"


#include <super_core/config.hpp>
#include <ros_interface/ros1/ros1_interface.hpp>
#include <data_structure/base/trajectory.h>

#include <data_structure/base/polytope.h>


#include "traj_opt/exp_traj_optimizer_s4.h"
#include "traj_opt/backup_traj_optimizer_s4.h"
#include "path_search/astar.h"
#include "rog_map/rog_map.h"
#include "super_core/corridor_generator.h"
#include "super_core/fov_checker.h"

#include "traj_opt/yaw_traj_opt.h"
#include "super_core/super_ret_code.hpp"
#include "utils/header/fmt_eigen.hpp"

#include <super_core/log_utils.hpp>
#include <data_structure/exp_traj.h>
#include <data_structure/cmd_traj.h>
#include <data_structure/backup_traj.h>


namespace super_planner {
    using namespace color_text;
    using namespace geometry_utils;

    class SuperPlanner {
        LogOneReplan latest_replan;
        super_planner::Config cfg_;
        rog_map::ROGMapROS::Ptr map_ptr_;
        CorridorGenerator::Ptr cg_ptr_;
        path_search::Astar::Ptr astar_ptr_;
        ros_interface::RosInterface::Ptr ros_ptr_;
        Vec3f shifted_sfc_start_pt_;

        traj_opt::ExpTrajOpt::Ptr exp_traj_opt_;
        traj_opt::BackupTrajOpt::Ptr back_traj_opt_;
        traj_opt::YawTrajOpt::Ptr yaw_traj_opt_;

        CIRI::Ptr ciri_;

        super_utils::RobotState robot_state_;

        std::mutex drone_state_mutex_;
        std::mutex replan_lock_;

        Vec3f local_start_p_;

        bool robot_on_backup_traj_{false};
        // use negative value to indicate the traj is not available
        double on_backup_start_WT{-1}, on_backup_end_WT{-1};

        double planner_process_start_WT_;

        struct GoalInfo {
            Vec3f goal_p{0, 0, 0};
            double goal_yaw{0};
            bool new_goal{true};
            bool goal_valid{true};
        } gi_;

        FOVChecker::Ptr fov_checker_;

        CmdTraj cmd_traj_info_;
        ExpTraj last_exp_traj_info_;

        vector<double> time_consuming_;

        /* Independent yaw reference (/super/goal_yaw).  The subscriber only
         * stores the latest target; the FSM timer splices it into the
         * committed yaw trajectory. */
        std::mutex yaw_state_mutex_;
        std::mutex yaw_ref_mutex_;
        std::mutex yaw_traj_mutex_;

        bool yaw_ref_received_{false};
        double yaw_ref_target_{0.0};
        double yaw_ref_receipt_WT_{0.0};
        double yaw_reference_timeout_s_{0.5};
        uint64_t yaw_ref_seq_{0};
        uint64_t yaw_ref_applied_seq_{0};
        uint64_t yaw_applied_pos_version_{0};
        bool yaw_timeout_applied_{true};
        bool yaw_traj_updated_{false};

        Trajectory idle_yaw_traj_{};
        bool idle_yaw_active_{false};
        bool idle_yaw_settling_{false};

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        explicit SuperPlanner(const std::string &cfg_path,
                              const ros_interface::RosInterface::Ptr &ros_ptr,
                              const rog_map::ROGMapROS::Ptr &map_ptr);

        ~SuperPlanner() = default;

        void lockCommittedTraj() {
            cmd_traj_info_.lock();
        }

        void unlockCommittedTraj() {
            cmd_traj_info_.unlock();
        }

        bool goalValid() const {
            return gi_.goal_valid;
        }

        typedef std::shared_ptr<SuperPlanner> Ptr;

        void getOneHeartbeatTime(double &start_WT_pos, bool &traj_finish);

        Trajectory getCommittedPositionTrajectory();

        Trajectory getCommittedYawTrajectory();

        void getOneCommandFromTraj(StatePVAJ &pvaj,
                                   double &yaw,
                                   double &yaw_dot,
                                   bool &on_backup_traj,
                                   bool &traj_finish);

        void getModuleTimeConsuming(vector<double> &time);

        /* Independent yaw reference.  All methods are safe to call from ROS
         * callback threads; the FSM timer drives updateYawReference(). */
        void setYawReference(const double &yaw, const double &receipt_WT);

        void setYawReferenceTimeout(const double &timeout_s);

        void updateYawReference();

        bool consumeYawTrajectoryUpdate();

        bool getOneIdleYawCommand(StatePVAJ &pvaj, double &yaw, double &yaw_dot);

        /* Tow type of replan strategy */
        RET_CODE PlanFromRest(const Vec3f &goal_p,
                              const double &goal_yaw,
                              const bool &new_goal);

        RET_CODE
        ReplanOnce(const Vec3f &goal_p,
                   const double &goal_yaw,
                   const bool &new_goal);

    private:
        RET_CODE generateExpTraj(ExpTraj &last_exp_traj_info,
                                 ExpTraj &out_exp_traj_info);

        /* For Backup traj generation */
        RET_CODE generateBackupTrajectory(ExpTraj &ref_exp_traj, BackupTraj &back_traj_info);

        int getNearestFurtherGoalPoint(const vec_E<Vec3f> &goals, const Vec3f &start_pt);

        bool PathSearch(const Vec3f &start_pt, const Vec3f &goal,
                        const double &searching_horizon,
                        vec_Vec3f &path);

        bool buildYawTargetSegment(const Vec4f &init_state,
                                   const double &target_yaw,
                                   const double &min_duration,
                                   Trajectory &out_traj);

        bool buildDefaultYawSegment(const Vec4f &init_state,
                                    const Trajectory &pos_remaining,
                                    Trajectory &out_traj);

        bool buildYawHoldSegment(const double &yaw,
                                 const double &duration,
                                 Trajectory &out_traj);

        bool commitYawTrajectory(const Trajectory &prefix,
                                 const Trajectory &segment,
                                 const double &start_WT,
                                 const double &pos_total,
                                 const uint64_t &pos_version);

        /* Commit a new position trajectory while carrying the active yaw
         * reference across the replan instead of falling back to the default
         * yaw planned for the new position trajectory. */
        void commitPositionTrajectory(const ExpTraj &exp_traj,
                                      const BackupTraj *backup_traj);

        void applyActiveYawReference(const Vec4f &init_state);

        Vec4f committedYawStateAt(const double &wall_time);

        bool yawReferenceActive(double &target_yaw, uint64_t &ref_seq);

        void updateIdleYawReference(const bool &valid_reference,
                                    const double &target_yaw,
                                    const uint64_t &ref_seq,
                                    const double &now);


    public:
        void getRobotState(rog_map::RobotState &out);

        bool isEasyGoal(const Vec3f &goal_position);

        rog_map::ROGMapROS::Ptr &getMap() {
            return map_ptr_;
        }

        double ft{0}, bt{0};
        int ft_cnt{0}, bt_cnt{0};

        double getFrontendTime() {
            if (ft_cnt == 0) return -1;
            double ave_t = ft / ft_cnt;
            ft = 0;
            ft_cnt = 0;
            return ave_t;
        }

        double getBackendTime() {
            if (bt_cnt == 0) return -1;
            double ave_t = bt / bt_cnt;
            bt = 0;
            bt_cnt = 0;
            return ave_t;
        }

        void updateROGMap(const rog_map::PointCloud &cloud, const super_utils::Pose &pose) const {
            map_ptr_->updateMap(cloud, pose);
        }

        LogOneReplan getLatestReplanLog() {
            latest_replan.setSfcPc(cg_ptr_->getLatestCloud());
            latest_replan.setComptT(time_consuming_);
            return latest_replan;
        }
    };
}
