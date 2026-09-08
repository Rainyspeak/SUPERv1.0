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


#ifndef SUPER_MISSION_WAYPOINT_MANAGER_HPP
#define SUPER_MISSION_WAYPOINT_MANAGER_HPP

#include <ros/ros.h>

#include <fstream>
#include <functional>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>

#include <utils/header/color_msg_utils.hpp>
#include <utils/header/color_text.hpp>
#include <utils/header/eigen_alias.hpp>

namespace mission {
    using namespace super_utils;
    using namespace color_text;
    using namespace std;

    /**
     * Waypoint mission manager, merged from the standalone mission_planner package.
     *
     * Waypoint sources (Config::waypoint_type):
     *   - "rviz": collect waypoints with the RViz "Publish Point" tool (/clicked_point).
     *             Starts automatically after waypoint_num points; waypoint_num <= 0
     *             collects until a message arrives on ~mission/trigger.
     *   - "file": load waypoints from a text file ("x y z switch_dis" per line),
     *             start on ~mission/trigger.
     *
     * Waypoints are dispatched to the fsm in-process through GoalSetter
     * (setGoalPosiAndYaw), never over a topic. Clicking during execution appends
     * the point to the end of the queue; clicking after DONE starts a fresh mission.
     */
    class WaypointManager {
    public:
        enum MissionState {
            IDLE = 0,
            COLLECTING,
            READY,
            EXECUTING,
            DONE
        };

        struct Config {
            bool enable{false};
            std::string waypoint_type{"rviz"};
            int waypoint_num{0};
            double switch_dis{1.0};
            std::string waypoint_file{""};
            std::string frame_id{"world"};
        };

        typedef std::function<void(const Vec3f &, const Quatf &)> GoalSetter;

        WaypointManager() = default;

        void init(const ros::NodeHandle &nh, const Config &cfg, GoalSetter set_goal) {
            nh_ = nh;
            cfg_ = cfg;
            set_goal_ = std::move(set_goal);

            state_pub_ = nh_.advertise<std_msgs::Int32>("mission/state", 10, true);
            path_pub_ = nh_.advertise<nav_msgs::Path>("mission/waypoints", 1, true);
            mkr_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("mission/markers", 1, true);

            if (cfg_.waypoint_type == "rviz") {
                click_sub_ = nh_.subscribe("/clicked_point", 10,
                                           &WaypointManager::clickedPointCallback, this);
                state_ = COLLECTING;
                cout << YELLOW << " -- [Mission] RViz collect mode: publish points on /clicked_point, "
                     << (cfg_.waypoint_num > 0
                         ? "auto start after " + to_string(cfg_.waypoint_num) + " points."
                         : "start by a message on ~mission/trigger.")
                     << RESET << endl;
            } else if (cfg_.waypoint_type == "file") {
                if (!loadWaypointsFromFile()) {
                    cfg_.enable = false;
                    return;
                }
                state_ = READY;
                cout << YELLOW << " -- [Mission] Loaded " << wps_.size()
                     << " waypoints from file, waiting for a message on ~mission/trigger." << RESET << endl;
            } else {
                cout << YELLOW << " -- [Mission] Unknown waypoint_type '" << cfg_.waypoint_type
                     << "', mission disabled." << RESET << endl;
                cfg_.enable = false;
                return;
            }

            trigger_sub_ = nh_.subscribe("mission/trigger", 1,
                                         &WaypointManager::triggerCallback, this);
            publishViz();
        }

        bool enabled() const { return cfg_.enable; }

        MissionState state() const { return state_; }

        /// Called from the fsm main timer (100 Hz). planner_ready means odom received
        /// and the fsm has left INIT, so that setGoalPosiAndYaw can be accepted.
        void onTick(const Vec3f &cur_p, const bool planner_ready) {
            if (!cfg_.enable || state_ != EXECUTING || !planner_ready) {
                return;
            }
            if (!goal_set_) {
                dispatchCurrentGoal();
                return;
            }
            if ((wps_[cur_idx_] - cur_p).norm() < switch_dis_[cur_idx_]) {
                cout << YELLOW << " -- [Mission] Close to waypoint " << cur_idx_
                     << ", switch to next." << RESET << endl;
                if (cur_idx_ + 1 < wps_.size()) {
                    cur_idx_++;
                    dispatchCurrentGoal();
                } else {
                    state_ = DONE;
                    publishState(-1);
                    cout << GREEN << " -- [Mission] All waypoints reached, hover at the last one."
                         << RESET << endl;
                }
            }
        }

    private:
        void clickedPointCallback(const geometry_msgs::PointStampedConstPtr &msg) {
            const Vec3f p(msg->point.x, msg->point.y, msg->point.z);
            if (state_ == DONE) {
                // a click after a finished mission starts a fresh collection
                wps_.clear();
                switch_dis_.clear();
                cur_idx_ = 0;
                goal_set_ = false;
                state_ = COLLECTING;
                cout << YELLOW << " -- [Mission] Previous mission finished, restart collecting."
                     << RESET << endl;
            }
            if (state_ != COLLECTING && state_ != EXECUTING) {
                return;
            }
            wps_.push_back(p);
            switch_dis_.push_back(cfg_.switch_dis);
            cout << YELLOW << " -- [Mission] "
                 << (state_ == EXECUTING ? "Append inflight" : "Collected")
                 << " waypoint " << wps_.size() - 1 << " at " << p.transpose() << RESET << endl;
            if (state_ == COLLECTING && cfg_.waypoint_num > 0 &&
                static_cast<int>(wps_.size()) >= cfg_.waypoint_num) {
                startMission();
            }
            publishViz();
        }

        void triggerCallback(const geometry_msgs::PoseStampedConstPtr & /*msg*/) {
            if ((state_ == READY || state_ == COLLECTING) && !wps_.empty()) {
                startMission();
            } else if (wps_.empty()) {
                cout << YELLOW << " -- [Mission] Trigger received but no waypoint yet." << RESET << endl;
            }
        }

        /// Official mission_planner waypoint file format: "x y z switch_dis" per line.
        bool loadWaypointsFromFile() {
            if (cfg_.waypoint_file.empty()) {
                cout << YELLOW << " -- [Mission] waypoint_file not set." << RESET << endl;
                return false;
            }
            ifstream fin(cfg_.waypoint_file);
            if (!fin) {
                cout << YELLOW << " -- [Mission] Cannot open waypoint file: "
                     << cfg_.waypoint_file << RESET << endl;
                return false;
            }
            std::string line;
            while (std::getline(fin, line)) {
                std::istringstream iss(line);
                std::vector<std::string> tokens;
                for (std::string s; iss >> s;) {
                    tokens.push_back(s);
                }
                if (tokens.size() < 4) {
                    continue;
                }
                Vec3f wp;
                for (size_t i = 0; i < 3; i++) {
                    wp(i) = std::stod(tokens[i]);
                }
                wps_.push_back(wp);
                switch_dis_.push_back(std::stod(tokens[3]));
            }
            return !wps_.empty();
        }

        void startMission() {
            state_ = EXECUTING;
            cur_idx_ = 0;
            goal_set_ = false;
            cout << GREEN << " -- [Mission] Start flying " << wps_.size() << " waypoints." << RESET << endl;
            publishViz();
        }

        void dispatchCurrentGoal() {
            const Quatf q(1, 0, 0, 0); // yaw 0, identical to the standalone mission_planner
            set_goal_(wps_[cur_idx_], q);
            goal_set_ = true;
            publishState(cur_idx_);
        }

        void publishState(const int idx) {
            std_msgs::Int32 msg;
            msg.data = idx;
            state_pub_.publish(msg);
        }

        void publishViz() {
            if (wps_.empty()) {
                return;
            }
            nav_msgs::Path path;
            path.header.frame_id = cfg_.frame_id;
            path.header.stamp = ros::Time::now();

            visualization_msgs::MarkerArray mkrs;
            visualization_msgs::Marker line;
            line.header = path.header;
            line.ns = "mission";
            line.id = 0;
            line.type = visualization_msgs::Marker::LINE_STRIP;
            line.action = visualization_msgs::Marker::ADD;
            line.pose.orientation.w = 1.0;
            line.scale.x = 0.1;
            line.color = Color::SteelBlue();

            for (size_t i = 0; i < wps_.size(); i++) {
                geometry_msgs::Point gp;
                gp.x = wps_[i].x();
                gp.y = wps_[i].y();
                gp.z = wps_[i].z();
                line.points.push_back(gp);

                geometry_msgs::PoseStamped pose;
                pose.header = path.header;
                pose.pose.position = gp;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);

                visualization_msgs::Marker ball;
                ball.header = path.header;
                ball.ns = "mission_pt";
                ball.id = i;
                ball.type = visualization_msgs::Marker::SPHERE;
                ball.action = visualization_msgs::Marker::ADD;
                ball.pose.orientation.w = 1.0;
                ball.pose.position = gp;
                ball.scale.x = ball.scale.y = ball.scale.z = 0.3;
                if (state_ == EXECUTING && static_cast<int>(i) == cur_idx_) {
                    ball.color = Color::Green();
                } else if (state_ == EXECUTING || state_ == DONE) {
                    if (static_cast<int>(i) < cur_idx_) {
                        ball.color = Color::Gray();
                        ball.color.a = 0.4;
                    } else {
                        ball.color = Color::SteelBlue();
                    }
                } else {
                    ball.color = Color::SteelBlue();
                }
                mkrs.markers.push_back(ball);

                visualization_msgs::Marker text;
                text.header = path.header;
                text.ns = "mission_id";
                text.id = i;
                text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                text.action = visualization_msgs::Marker::ADD;
                text.pose.orientation.w = 1.0;
                text.pose.position = gp;
                text.pose.position.z += 0.5;
                text.scale.z = 0.6;
                text.color = Color::Black();
                text.text = to_string(i);
                mkrs.markers.push_back(text);
            }
            mkrs.markers.push_back(line);
            path_pub_.publish(path);
            mkr_pub_.publish(mkrs);
        }

        Config cfg_;
        MissionState state_{IDLE};
        std::vector<Vec3f> wps_;
        std::vector<double> switch_dis_;
        int cur_idx_{0};
        bool goal_set_{false};
        GoalSetter set_goal_;

        ros::NodeHandle nh_;
        ros::Subscriber click_sub_, trigger_sub_;
        ros::Publisher state_pub_, path_pub_, mkr_pub_;
    };
}

#endif //SUPER_MISSION_WAYPOINT_MANAGER_HPP
