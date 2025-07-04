////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2019
// Commonwealth Scientific and Industrial Research Organisation (CSIRO)
// ABN 41 687 119 230
//
// Author: Fletcher Talbot
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include "state_controller.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

StateController::StateController(void) {

    // Get parameters from parameter server and initialises parameter map
    initParameters();

    // Create robot model
    model_ = std::allocate_shared<Model>(Eigen::aligned_allocator<Model>(), params_);
    model_->generate();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

StateController::~StateController(void) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::init(void) {
    // Set initial gait selection number for gait toggling
    if (params_.gait_type.data == "tripod_gait") {
        gait_selection_ = TRIPOD_GAIT;
    } else if (params_.gait_type.data == "ripple_gait") {
        gait_selection_ = RIPPLE_GAIT;
    } else if (params_.gait_type.data == "wave_gait") {
        gait_selection_ = WAVE_GAIT;
    } else if (params_.gait_type.data == "amble_gait") {
        gait_selection_ = AMBLE_GAIT;
    }

    // Create controller objects and smart pointers
    walker_ = std::allocate_shared<WalkController>(Eigen::aligned_allocator<WalkController>(), model_, params_);
    walker_->init();
    poser_ = std::allocate_shared<PoseController>(Eigen::aligned_allocator<PoseController>(), model_, params_);
    poser_->init();
    admittance_ =
        std::allocate_shared<AdmittanceController>(Eigen::aligned_allocator<AdmittanceController>(), model_, params_);

    robot_state_ = UNKNOWN;

    initialised_ = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::loop(void) {
    // Posing - updates currentPose for body compensation
    if (robot_state_ != UNKNOWN) {
        poser_->updateCurrentPose(robot_state_);
        walker_->setPoseState(poser_->getAutoPoseState()); // Sends pose state from poser to walker
        generateExternalTargetTransforms();

        // Admittance control - updates deltaZ values
        if (params_.admittance_control.data) {
            // Calculate new stiffness based on walking cycle
            if (walker_->getWalkState() != STOPPED && params_.dynamic_stiffness.data) {
                admittance_->updateStiffness(walker_);
            }
            admittance_->updateAdmittance();
        }
    }

    // Syropod state machine
    if (transition_state_flag_) {
        transitionRobotState();
    }

    if (robot_state_ == RUNNING) {
        runningState();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::transitionRobotState(void) {
    // UNKNOWN -> OFF/RPACKED/READY/RUNNING
    if (robot_state_ == UNKNOWN) {
        // Check how many joints/legs are in the packed state
        int legs_packed = 0;
        int legs_ready = 0;
        for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_) {
            std::shared_ptr<Leg> leg = leg_it_->second;
            int joints_packed = 0;
            int joints_ready = 0;
            for (joint_it_ = leg->getJointContainer()->begin(); joint_it_ != leg->getJointContainer()->end(); ++joint_it_) {
                std::shared_ptr<Joint> joint = joint_it_->second;
                joints_packed += int(abs(joint->current_position_ - joint->packed_positions_.back()) < JOINT_TOLERANCE);
                joints_ready += int(abs(joint->current_position_ - joint->unpacked_position_) < JOINT_TOLERANCE);
            }
            legs_packed += int(joints_packed == leg->getJointCount());
            legs_ready += int(joints_ready == leg->getJointCount());
        }

        // Syropod estimated to be in RPACKED state
        if (legs_packed == model_->getLegCount()) {
            if (!params_.start_up_sequence.data) {
                printf("\nSyropod currently in packed state and cannot run direct startup sequence.\n"
                       "Either manually unpack Syropod or set start_up_sequence to true in config file\n");
            } else {
                robot_state_ = RPACKED;
                printf("\nSyropod currently in RPACKED state. Press START to proceed to READY state.\n");
            }
        }
        // Syropod estimated to be in READY state
        if (legs_ready == model_->getLegCount()) {
            if (!params_.start_up_sequence.data) {
                robot_state_ = RPACKED;
                printf("\nSyropod currently in READY state but is set to execute DIRECT transition.\n"
                       "Ensure Syropod legs are not bearing load before proceeding.\n"
                       "Press START to proceed to RUNNING state.\n");
            } else {
                robot_state_ = READY;
                printf("\nSyropod currently in READY state. Press START to proceed to RUNNING state.\n");
            }
        }
        // Syropod state unknown
        else {
            robot_state_ = RPACKED;
            printf("\nCurrent Syropod state is undefined!\nFuture state transitions may be undesireable! "
                   "Ensure Syropod legs are not bearing load before proceeding.\nPress START to proceed.\n");
        }
        new_robot_state_ = robot_state_;
    }
    // RPACKED -> RUNNING (Direct)
    else if (robot_state_ == RPACKED && new_robot_state_ == READY && !params_.start_up_sequence.data) {
        int progress = poser_->directStartup();
        double throttle = params_.time_to_start.data / 5.0;
        printf("\n[SHC] Syropod transitioning DIRECTLY to RUNNING state (%d%%). . .\n", progress);
        if (progress == PROGRESS_COMPLETE) {
            printf("\n[SHC] Syropod transitioning DIRECTLY to RUNNING state (%d%%). . .\n", progress);
            robot_state_ = READY;
            model_->updateDefaultConfiguration();
            model_->generateWorkspaces();
            walker_->generateWalkspace();
            printf("\nDirect startup sequence complete. Place Syropod on walking surface and press START to proceed.\n");
        }
    }
    // READY -> RUNNING (Direct)
    else if (robot_state_ == READY && new_robot_state_ == RUNNING && !params_.start_up_sequence.data) {
        robot_state_ = RUNNING;
        printf("\nReady to Walk.\n");
    }
    // RUNNING -> OTHER STATE (Direct)
    else if (robot_state_ == RUNNING && new_robot_state_ != RUNNING && !params_.start_up_sequence.data) {
        transition_state_flag_ = false;
        printf("\nSyropod cannot transition from RUNNING state."
               " Set start_up_sequence parameter true to enable that functionality.\n");
    }
    // RPACKED -> READY (Unpack Syropod)
    else if (robot_state_ == RPACKED && new_robot_state_ == READY) {
        int progress = poser_->unpackLegs(PACK_TIME / params_.step_frequency.current_value);
        printf("\nSyropod transitioning to READY state (%d%%). . .\n", progress);
        if (progress == PROGRESS_COMPLETE) {
            robot_state_ = READY;
            printf("\nState transition complete. Syropod is in READY state.\n");
        }
    }
    // READY -> RPACKED (Pack Syropod)
    else if (robot_state_ == READY && new_robot_state_ == RPACKED) {
        int progress = poser_->packLegs(PACK_TIME / params_.step_frequency.current_value);
        printf("\nSyropod transitioning to RPACKED state (%d%%). . .\n", progress);
        if (progress == PROGRESS_COMPLETE) {
            robot_state_ = RPACKED;
            printf("\nState transition complete. Syropod is in RPACKED state.\n");
        }
    }
    // READY -> RUNNING (Initate start up sequence to step to walking stance)
    else if (robot_state_ == READY && new_robot_state_ == RUNNING) {
        int progress = poser_->executeSequence(START_UP);
        std::string progress_string = (progress == -1 ? "Generating Sequence" : (numberToString(progress) + "%"));
        printf("\nSyropod transitioning to RUNNING state (%s). . .\n", progress_string.c_str());
        if (progress == PROGRESS_COMPLETE) {
            walker_->init();
            model_->updateDefaultConfiguration();
            model_->generateWorkspaces();
            walker_->generateWalkspace();
            robot_state_ = RUNNING;
            printf("\nState transition complete. Syropod is in RUNNING state. Ready to walk.\n");
        }
    }
    // RUNNING -> READY (Initiate shut down sequence to step from walking stance to ready stance)
    else if (robot_state_ == RUNNING && new_robot_state_ == READY) {
        // Force Syropod to stop walking
        if (walker_->getWalkState() == STOPPED) {
            // Return any manually controlled legs before executing shutdown
            if (manual_leg_count_ != 0) {
                if (primary_leg_selection_ != LEG_UNDESIGNATED) {
                    toggle_primary_leg_state_ = primary_leg_->getLegState() == MANUAL;
                }
                if (secondary_leg_selection_ != LEG_UNDESIGNATED) {
                    toggle_secondary_leg_state_ = (secondary_leg_->getLegState() == MANUAL);
                }
            } else {
                int progress = poser_->executeSequence(SHUT_DOWN);
                printf("\nSyropod transitioning to READY state (%d%%). . .\n", progress);
                if (progress == PROGRESS_COMPLETE) {
                    robot_state_ = READY;
                    printf("\nState transition complete. Syropod is in READY state.\n");
                }
            }
        } else {
            linear_velocity_input_ = Eigen::Vector2d::Zero();
            angular_velocity_input_ = 0.0;
            printf("\nStopping Syropod to transition state . . .\n");
        }
    }
    // Undefined system transition
    else {
        // Fatal error
        printf("\nUndefined system state transition was requested! Shutting down controller!\n");
    }

    // Transition complete
    if (robot_state_ == new_robot_state_) {
        transition_state_flag_ = false;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::runningState(void) {
    bool update_tip_position = true;

    // Force Syropod to stop walking
    if (transition_state_flag_) {
        transitionRobotState();
        update_tip_position = false;
    }
    // Switch gait and update walker parameters
    else if (gait_change_flag_) {
        changeGait();
        update_tip_position = false;
    }
    // Toggle state of leg and transition between states
    else if (toggle_primary_leg_state_ || toggle_secondary_leg_state_) {
        legStateToggle();
        update_tip_position = false;
    } else if (planner_mode_ == PLANNER_MODE_ON) {
        executePlan();
        update_tip_position = false;
    }
    // Cruise control (constant velocity input)
    else if (cruise_control_mode_ == CRUISE_CONTROL_ON &&
             (params_.cruise_control_time_limit.data == 0.0
              // || ros::Time::now().toSec() < cruise_control_end_time_ //TODO
              )) {
        linear_velocity_input_ = linear_cruise_velocity_;
        angular_velocity_input_ = angular_cruise_velocity_;
    }

    // Dynamically adjust parameters and change stance if required
    if (parameter_adjust_flag_) {
        adjustParameter();
    }

    // Set true if already true or if walk state not STOPPED
    update_tip_position = update_tip_position || walker_->getWalkState() != STOPPED;

    // Update tip positions unless Syropod is undergoing state transition, gait switch, parameter adjustment or
    // leg state transition (which all only occur once the Syropod has stopped walking)
    if (update_tip_position) {
        // Update tip positions for walking legs
        walker_->updateWalk(linear_velocity_input_, angular_velocity_input_);

        // Update tip positions for manually controlled legs
        walker_->updateManual(primary_leg_selection_, primary_tip_velocity_input_,
                              secondary_leg_selection_, secondary_tip_velocity_input_);

        // Controls tip position for manually controlled legs.
        // Primary leg correspond to the front right leg and secondary leg is the front left leg.
        // TODO: give access for the remaindering legs this feature if selected.
        walker_->updateManual(primary_leg_selection_, primary_pose_input_,
                              secondary_leg_selection_, secondary_pose_input_);

        // Pose controller takes current tip positions from walker and applies body posing
        poser_->updateStance();

        // Model takes desired tip poses from pose controller and applies inverse/forwards kinematics
        model_->updateModel();
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::adjustParameter(void) {
    AdjustableParameter *p = dynamic_parameter_;
    p->current_value = new_parameter_value_;
    bool set_new_parameter = true;
    if (p->name == "step_frequency") {
        // Calculate new speed/acceleration limits due to changing parameter
        StepCycle new_step_cycle = walker_->generateStepCycle(false);
        LimitMap max_linear_speed_map;
        LimitMap max_angular_speed_map;
        walker_->generateLimits(new_step_cycle, &max_linear_speed_map, &max_angular_speed_map);
        walker_->setLinearSpeedLimitMap(max_linear_speed_map);
        walker_->setAngularSpeedLimitMap(max_angular_speed_map);
        double max_linear_speed =
            walker_->getLimit(linear_velocity_input_, angular_velocity_input_, max_linear_speed_map);
        double max_angular_speed =
            walker_->getLimit(linear_velocity_input_, angular_velocity_input_, max_angular_speed_map);

        // Generate target velocities to achieve before changing step frequency
        Eigen::Vector2d target_linear_velocity;
        double target_angular_velocity;
        if (params_.velocity_input_mode.data == "throttle") {
            target_linear_velocity = clamped(linear_velocity_input_, 1.0) * max_linear_speed; // Forces input between -1.0/1.0
            target_angular_velocity = clamped(angular_velocity_input_, -1.0, 1.0) * max_angular_speed;

            // Scale linear velocity according to angular velocity (% of max) to keep stride velocities within limits
            target_linear_velocity *= (1.0 - abs(angular_velocity_input_));
        } else if (params_.velocity_input_mode.data == "real") {
            target_linear_velocity = clamped(linear_velocity_input_, max_linear_speed);
            target_angular_velocity = clamped(angular_velocity_input_, -max_angular_speed, max_angular_speed);
        }

        // Set new parameter once within new limits
        if (walker_->getDesiredLinearVelocity()[0] <= target_linear_velocity[0] &&
            walker_->getDesiredLinearVelocity()[1] <= target_linear_velocity[1] &&
            abs(walker_->getDesiredAngularVelocity()) <= abs(target_angular_velocity)) {
            walker_->generateStepCycle();
            walker_->generateLimits();
        } else {
            set_new_parameter = false;
            printf(
                "\n[SHC] Slowing to safe speed before setting new parameter '%s'\n", p->name.c_str());
        }
    }

    if (set_new_parameter) {
        parameter_adjust_flag_ = false;
        printf("\n[SHC] Parameter '%s' set to %f. (Default: %f, Min: %f, Max: %f)\n",
               p->name.c_str(), p->current_value, p->default_value, p->min_value, p->max_value);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::changeGait(void) {
    if (walker_->getWalkState() == STOPPED) {
        initGaitParameters(gait_selection_);
        walker_->generateStepCycle();
        walker_->generateLimits();

        // For auto compensation find associated auto posing parameters for new gait
        if (params_.auto_posing.data && params_.auto_pose_type.data == "auto") {
            initAutoPoseParameters();
            poser_->setAutoPoseParams();
        }

        gait_change_flag_ = false;
        printf("\nNow using %s mode.\n", params_.gait_type.data.c_str());
    }
    // Force Syropod to stop walking
    else {
        linear_velocity_input_ = Eigen::Vector2d::Zero();
        angular_velocity_input_ = 0.0;
        printf("\nStopping Syropod to change gait . . .\n");
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::legStateToggle(void) {
    if (walker_->getWalkState() == STOPPED) {
        // Choose primary or secondary leg state to transition
        std::shared_ptr<Leg> leg; // Transitioning leg
        std::shared_ptr<LegState> new_leg_state;
        if (toggle_primary_leg_state_) {
            leg = model_->getLegByIDNumber(primary_leg_selection_);
            new_leg_state = std::make_shared<LegState>(primary_leg_state_);
        } else if (toggle_secondary_leg_state_) {
            leg = model_->getLegByIDNumber(secondary_leg_selection_);
            new_leg_state = std::make_shared<LegState>(secondary_leg_state_);
        }
        std::string leg_name = leg->getIDName();

        // Calculate default pose for new loading pattern
        // poser_->calculateDefaultPose();

        // Set new leg state and transition position if required
        // WALKING -> WALKING_TO_MANUAL
        if (leg->getLegState() == WALKING) {
            if (manual_leg_count_ < MAX_MANUAL_LEGS) {
                printf(
                    "\n%s leg transitioning to MANUAL state . . .\n",
                    leg->getIDName().c_str());
                leg->setLegState(WALKING_TO_MANUAL);
                std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
                leg_stepper->setSwingProgress(-1.0);
                leg_stepper->setStanceProgress(-1.0);
            } else {
                printf("\nOnly allowed to have %d legs manually manipulated at one time.\n", MAX_MANUAL_LEGS);
                toggle_primary_leg_state_ = false;
                toggle_secondary_leg_state_ = false;
            }
        }
        // MANUAL -> MANUAL_TO_WALKING
        else if (leg->getLegState() == MANUAL) {
            printf(
                "\n%s leg transitioning to WALKING state . . .\n",
                leg->getIDName().c_str());
            leg->setLegState(MANUAL_TO_WALKING);
        }
        // WALKING_TO_MANUAL -> MANUAL
        else if (leg->getLegState() == WALKING_TO_MANUAL) {
            poser_->setPoseResetMode(IMMEDIATE_ALL_RESET); // Set to ALL_RESET to force pose to new default pose
            int progress = poser_->poseForLegManipulation();

            // Update stiffness for transition to MANUAL state
            if (params_.dynamic_stiffness.data) {
                double scale_reference = double(progress) / PROGRESS_COMPLETE; // 0.0->1.0
                admittance_->updateStiffness(leg, scale_reference);
            }

            if (progress == PROGRESS_COMPLETE) {
                leg->setLegState(MANUAL);
                *new_leg_state = MANUAL;
                printf("\n%s leg set to state: MANUAL.\n", leg->getIDName().c_str());
                toggle_primary_leg_state_ = false;
                toggle_secondary_leg_state_ = false;
                poser_->setPoseResetMode(NO_RESET);
                manual_leg_count_++;
            }
        }
        // MANUAL_TO_WALKING -> WALKING
        else if (leg->getLegState() == MANUAL_TO_WALKING) {
            poser_->setPoseResetMode(IMMEDIATE_ALL_RESET); // Set to ALL_RESET to force pose to new default pose
            int progress = poser_->poseForLegManipulation();

            // Update stiffness for transition to WALKING state
            if (params_.dynamic_stiffness.data) {
                double scale_reference = abs(double(progress) / PROGRESS_COMPLETE - 1.0); // 1.0->0.0
                admittance_->updateStiffness(leg, scale_reference);
            }

            if (progress == PROGRESS_COMPLETE) {
                leg->setLegState(WALKING);
                *new_leg_state = WALKING;
                printf("\n%s leg set to state: WALKING.\n", leg->getIDName().c_str());
                toggle_primary_leg_state_ = false;
                toggle_secondary_leg_state_ = false;
                poser_->setPoseResetMode(NO_RESET);
                manual_leg_count_--;
            }
        }
    }
    // Force Syropod to stop walking
    else {
        printf("\nStopping Syropod to transition leg state . . .\n");
        linear_velocity_input_ = Eigen::Vector2d::Zero();
        angular_velocity_input_ = 0.0;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::executePlan(void) {
    if (walker_->getWalkState() == STOPPED) {
        int progress;

        // Publish request for step N of plan
        if (!target_configuration_acquired_ && !target_tip_pose_acquired_ && !target_body_pose_acquired_) {
            printf("\n[SHC]\tWaiting for plan step %d to be published . . .\n", plan_step_);
            model_->updateModel();
        } else {
            // Execute acquired plan step and iterate to next on completion
            if (target_configuration_acquired_) {
                progress = poser_->transitionConfiguration(5.0);
            } else if (target_tip_pose_acquired_ || target_body_pose_acquired_) {
                progress = poser_->transitionStance(5.0);
            }
            printf("\n[SHC]\tPlan step %d acquired. Executing (%d%%) . . .\n",
                   plan_step_, progress);
            if (progress == PROGRESS_COMPLETE) {
                printf("\n[SHC]\tPlan step %d completed.\n", plan_step_);
                plan_step_++;
                poser_->setTargetBodyPose(Pose::Identity());
                target_configuration_acquired_ = false;
                target_tip_pose_acquired_ = false;
                target_body_pose_acquired_ = false;
            }
        }
    }
    // Force Syropod to stop walking
    else {
        printf("\nStopping Syropod to begin plan execution . . .\n");
        linear_velocity_input_ = Eigen::Vector2d(0.0, 0.0);
        angular_velocity_input_ = 0.0;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::publishDesiredJointState(void) {
    sensor_msgs::JointState joint_state_msg;
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_) {
        std::shared_ptr<Leg> leg = leg_it_->second;
        JointContainer::iterator joint_it;
        if (params_.combined_control_interface.data) {
            leg->generateDesiredJointStateMsg(&joint_state_msg);
        }

        if (params_.individual_control_interface.data) {
            for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it) {
                std::shared_ptr<Joint> joint = joint_it->second;

                double postion = joint->desired_position_ + joint->offset_;
                // TODO: Show Desired Joint State
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::publishLegState(void) {

    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_) {
        syropod_highlevel_controller::LegState msg;
        std::shared_ptr<Leg> leg = leg_it_->second;
        std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
        std::shared_ptr<LegPoser> leg_poser = leg->getLegPoser();

        msg.name = leg->getIDName().c_str();

        // Tip poses

        msg.walker_tip_pose.pose = leg_stepper->getCurrentTipPose().toPoseMessage();


        msg.target_tip_pose.pose = leg_stepper->getTargetTipPose().toPoseMessage();


        msg.poser_tip_pose.pose = leg_poser->getCurrentTipPose().toPoseMessage();


        msg.model_tip_pose.pose = leg->getCurrentTipPose().toPoseMessage();


        msg.actual_tip_pose.pose = leg->applyFK(false, true).toPoseMessage();
        leg->applyFK();

        // Tip velocities

        msg.model_tip_velocity.twist.linear.x = leg->getCurrentTipVelocity()[0];
        msg.model_tip_velocity.twist.linear.y = leg->getCurrentTipVelocity()[1];
        msg.model_tip_velocity.twist.linear.z = leg->getCurrentTipVelocity()[2];

        // Joint positions/velocities
        for (joint_it_ = leg->getJointContainer()->begin(); joint_it_ != leg->getJointContainer()->end(); ++joint_it_) {
            std::shared_ptr<Joint> joint = joint_it_->second;
            msg.joint_positions.push_back(joint->desired_position_);
            msg.joint_velocities.push_back(joint->desired_velocity_);
            msg.joint_efforts.push_back(joint->desired_effort_);
        }

        // Step progress
        msg.swing_progress = leg_stepper->getSwingProgress();
        msg.stance_progress = leg_stepper->getStanceProgress();
        StepCycle step = walker_->getStepCycle();
        double swing_time = (double(step.swing_period_) / step.period_) / step.frequency_;
        double stance_time = (double(step.stance_period_) / step.period_) / step.frequency_;
        double time_to_swing_end;
        if (leg_stepper->getStanceProgress() >= 0.0) {
            time_to_swing_end = stance_time * (1.0 - leg_stepper->getStanceProgress()) + swing_time;
        } else {
            time_to_swing_end = swing_time * (1.0 - leg_stepper->getSwingProgress());
        }
        msg.time_to_swing_end = time_to_swing_end;
        msg.pose_delta = walker_->calculateOdometry(time_to_swing_end).toPoseMessage();

        // Leg specific auto pose
        Eigen::Vector3d position = leg_poser->getAutoPose().position_;
        Eigen::Quaterniond rotation = leg_poser->getAutoPose().rotation_;
        msg.auto_pose = Pose(position, rotation).toPoseMessage();

        // Admittance controller
        msg.tip_force.x = leg->getTipForceCalculated()[0] * params_.force_gain.current_value;
        msg.tip_force.y = leg->getTipForceCalculated()[1] * params_.force_gain.current_value;
        msg.tip_force.z = leg->getTipForceCalculated()[2] * params_.force_gain.current_value;
        msg.admittance_delta.x = leg->getAdmittanceDelta()[0];
        msg.admittance_delta.y = leg->getAdmittanceDelta()[1];
        msg.admittance_delta.z = leg->getAdmittanceDelta()[2];
        msg.virtual_stiffness = leg->getVirtualStiffness();

        // TODO: Show Leg State
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::publishVelocity(void) {
    geometry_msgs::Twist msg;
    msg.linear.x = walker_->getDesiredLinearVelocity()[0];
    msg.linear.y = walker_->getDesiredLinearVelocity()[1];
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = walker_->getDesiredAngularVelocity();
    // TODO: Show Velocity
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::publishPose(void) {
    geometry_msgs::Twist msg;
    Eigen::Vector3d position = model_->getCurrentPose().position_;
    Eigen::Quaterniond rotation = model_->getCurrentPose().rotation_;
    msg.linear.x = position[0];
    msg.linear.y = position[1];
    msg.linear.z = position[2];
    msg.angular.x = quaternionToEulerAngles(rotation)[0];
    msg.angular.y = quaternionToEulerAngles(rotation)[1];
    msg.angular.z = quaternionToEulerAngles(rotation)[2];
    // TODO: Show Pose
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::publishWalkspace(void) {
    // if (robot_state_ == RUNNING) {
    //     std_msgs::Float32MultiArray msg;
    //     LimitMap walkspace_map = walker_->getWalkspace();
    //     LimitMap::iterator walkspace_it;
    //     for (walkspace_it = walkspace_map.begin(); walkspace_it != walkspace_map.end(); ++walkspace_it) {
    //         msg.data.push_back(static_cast<float>(walkspace_it->second));
    //     }

    //     walkspace_publisher_.publish(msg);
    // }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::publishRotationPoseError(void) {
    // std_msgs::Float32MultiArray msg;
    // msg.data.clear();
    // msg.data.push_back(static_cast<float>(poser_->getRotationAbsementError()[0]));
    // msg.data.push_back(static_cast<float>(poser_->getRotationAbsementError()[1]));
    // msg.data.push_back(static_cast<float>(poser_->getRotationAbsementError()[2]));
    // msg.data.push_back(static_cast<float>(poser_->getRotationPositionError()[0]));
    // msg.data.push_back(static_cast<float>(poser_->getRotationPositionError()[1]));
    // msg.data.push_back(static_cast<float>(poser_->getRotationPositionError()[2]));
    // msg.data.push_back(static_cast<float>(poser_->getRotationVelocityError()[0]));
    // msg.data.push_back(static_cast<float>(poser_->getRotationVelocityError()[1]));
    // msg.data.push_back(static_cast<float>(poser_->getRotationVelocityError()[2]));
    // rotation_pose_error_publisher_.publish(msg);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::systemStateCallback(const int &input) {
    new_system_state_ = static_cast<SystemState>(input);
    if (system_state_ != new_system_state_) {
        system_state_ = new_system_state_;
        if (system_state_ == OPERATIONAL && initialised_) {
            printf("\nController operation resumed.\n");
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::robotStateCallback(const int &input) {
    RobotState input_state = static_cast<RobotState>(input);

    // Wait for any other transitions to complete
    bool ready_for_transition = !(toggle_primary_leg_state_ || toggle_secondary_leg_state_ || parameter_adjust_flag_);
    if (transition_state_flag_ && !ready_for_transition) {
        transition_state_flag_ = false;
    }

    // Handle single step transitioning between multiple states
    if (input_state != robot_state_ && !transition_state_flag_) {
        new_robot_state_ = input_state;
        if (new_robot_state_ > robot_state_) {
            new_robot_state_ = static_cast<RobotState>(robot_state_ + 1);
            transition_state_flag_ = ready_for_transition;
        } else if (new_robot_state_ < robot_state_) {
            new_robot_state_ = static_cast<RobotState>(robot_state_ - 1);
            transition_state_flag_ = ready_for_transition;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::bodyVelocityInputCallback(const geometry_msgs::Twist &input) {
    if (robot_state_ == RUNNING) {
        linear_velocity_input_ = Eigen::Vector2d(input.linear.x, input.linear.y) * params_.body_velocity_scaler.data;
        angular_velocity_input_ = input.angular.z * params_.body_velocity_scaler.data;
        if (params_.velocity_input_mode.data == "throttle" && linear_velocity_input_.norm() > 1.0) {
            linear_velocity_input_ = std::min(1.0, linear_velocity_input_.norm()) * linear_velocity_input_.normalized();
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::bodyPoseInputCallback(const geometry_msgs::Twist &input) {
    if (robot_state_ == RUNNING) {
        Eigen::Vector3d rotation_input(input.angular.x, input.angular.y, input.angular.z);
        Eigen::Vector3d translation_input(input.linear.x, input.linear.y, input.linear.z);
        poser_->setManualPoseInput(translation_input, rotation_input);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::posingModeCallback(const int &input) {
    if (robot_state_ == RUNNING) {
        PosingMode new_posing_mode = static_cast<PosingMode>(int(input));
        if (new_posing_mode != posing_mode_) {
            posing_mode_ = new_posing_mode;
            switch (posing_mode_) // Used only for user message, control handled by syropod_remote
            {
            case (NO_POSING):
                printf("\nPosing mode set to NO_POSING. "
                       "Body will not respond to manual posing input (except for reset commands).\n");
                break;
            case (X_Y_POSING):
                printf("\nPosing mode set to X_Y_POSING. "
                       "Body will only respond to x/y translational manual posing input.\n");
                break;
            case (PITCH_ROLL_POSING):
                printf("\nPosing mode set to PITCH_ROLL_POSING. "
                       "Body will only respond to pitch/roll rotational manual posing input.\n");
                break;
            case (Z_YAW_POSING):
                printf("\nPosing mode set to Z_YAW_POSING. "
                       "Body will only respond to z translational and yaw rotational manual posing input.\n");
                break;
            case (EXTERNAL_POSING):
                printf("\nPosing mode set to EXTERNAL_POSING. "
                       "Body will only respond to posing input from external source.\n");
                break;
            default:
                break;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::poseResetCallback(const int &input) {
    if (system_state_ != SUSPENDED && poser_ != NULL) {
        if (poser_->getPoseResetMode() != IMMEDIATE_ALL_RESET) {
            poser_->setPoseResetMode(static_cast<PoseResetMode>(input));
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::gaitSelectionCallback(const int &input) {
    if (robot_state_ == RUNNING) {
        GaitDesignation new_gait_selection = static_cast<GaitDesignation>(int(input));
        if (new_gait_selection != gait_selection_ && new_gait_selection != GAIT_UNDESIGNATED) {
            gait_selection_ = new_gait_selection;
            gait_change_flag_ = true;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::cruiseControlCallback(const int &input) {
    if (robot_state_ == RUNNING) {
        CruiseControlMode new_cruise_control_mode = static_cast<CruiseControlMode>(int(input));
        if (new_cruise_control_mode != cruise_control_mode_) {
            cruise_control_mode_ = new_cruise_control_mode;
            if (new_cruise_control_mode == CRUISE_CONTROL_ON) {
                // TODO: Obtener now() en segundos
                int now = 0;
                cruise_control_end_time_ = now + params_.cruise_control_time_limit.data;
                if (params_.force_cruise_velocity.data) {
                    // Set cruise velocity according to parameters
                    linear_cruise_velocity_[0] = params_.linear_cruise_velocity.data["x"] * params_.body_velocity_scaler.data;
                    linear_cruise_velocity_[1] = params_.linear_cruise_velocity.data["y"] * params_.body_velocity_scaler.data;
                    angular_cruise_velocity_ = params_.angular_cruise_velocity.data * params_.body_velocity_scaler.data;
                } else {
                    // Save current velocity input as cruise input
                    linear_cruise_velocity_ = linear_velocity_input_;
                    angular_cruise_velocity_ = angular_velocity_input_;
                }
                printf("\nCruise control ON - Input velocity set to constant: Linear(X:Y): %f:%f, Angular(Z): %f\n",
                       linear_cruise_velocity_[0], linear_cruise_velocity_[1], angular_cruise_velocity_);
            } else if (new_cruise_control_mode == CRUISE_CONTROL_EXTERNAL) {
                printf("\nCruise control EXTERNAL - Input velocity set by external node.\n");
            } else if (new_cruise_control_mode == CRUISE_CONTROL_OFF) {
                printf("\nCruise control OFF - Input velocity set by user.\n");
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::plannerModeCallback(const int &input) {
    if (robot_state_ == RUNNING) {
        PlannerMode new_planner_mode = static_cast<PlannerMode>(int(input));
        if (new_planner_mode != planner_mode_) {
            planner_mode_ = new_planner_mode;
            if (planner_mode_ == PLANNER_MODE_ON) {
                printf("\nPlanner mode ON. System will start executing plan published from planner.\n");
                plan_step_ = 0;
            } else {
                printf("\nPlanner mode OFF. ???.\n");
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::primaryLegSelectionCallback(const int &input) {
    if (robot_state_ == RUNNING) {
        LegDesignation new_primary_leg_selection = static_cast<LegDesignation>(input);
        if (primary_leg_selection_ != new_primary_leg_selection) {
            primary_leg_selection_ = new_primary_leg_selection;
            if (new_primary_leg_selection != LEG_UNDESIGNATED) {
                primary_leg_ = model_->getLegByIDNumber(primary_leg_selection_);
                printf("\n%s leg selected for primary control.\n", primary_leg_->getIDName().c_str());
            } else {
                printf("\nNo leg currently selected for primary control.\n");
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::secondaryLegSelectionCallback(const int &input) {
    if (robot_state_ == RUNNING) {
        LegDesignation new_secondary_leg_selection = static_cast<LegDesignation>(input);
        if (secondary_leg_selection_ != new_secondary_leg_selection) {
            secondary_leg_selection_ = new_secondary_leg_selection;
            if (new_secondary_leg_selection != LEG_UNDESIGNATED) {
                secondary_leg_ = model_->getLegByIDNumber(secondary_leg_selection_);
                printf("\n%s leg selected for secondary control.\n", secondary_leg_->getIDName().c_str());
            } else {
                printf("\nNo leg currently selected for secondary control.\n");
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::primaryLegStateCallback(const int &input) {
    if (robot_state_ == RUNNING && !transition_state_flag_) {
        LegState newPrimaryLegState = static_cast<LegState>(int(input));
        if (newPrimaryLegState != primary_leg_state_) {
            if (primary_leg_selection_ == LEG_UNDESIGNATED) {
                printf("\nCannot toggle primary leg state as no leg is currently selected as primary."
                       "\nPress left bumper to select a leg and try again.\n");
            } else if (toggle_secondary_leg_state_) {
                printf("\nCannot toggle primary leg state as secondary leg is currently "
                       "transitioning states.\nPlease wait and try again.\n");
            } else {
                primary_leg_state_ = newPrimaryLegState;
                toggle_primary_leg_state_ = true;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::secondaryLegStateCallback(const int &input) {
    if (robot_state_ == RUNNING && !transition_state_flag_) {
        LegState newSecondaryLegState = static_cast<LegState>(int(input));
        if (newSecondaryLegState != secondary_leg_state_) {
            if (secondary_leg_selection_ == LEG_UNDESIGNATED) {
                printf("\nCannot toggle secondary leg state as no leg is currently selected as secondary."
                       "\nPress right bumper to select a leg and try again.\n");
            } else if (toggle_primary_leg_state_) {
                printf("\nCannot toggle secondary leg state as primary leg is currently "
                       "transitioning states.\nPlease wait and try again.\n");
            } else {
                secondary_leg_state_ = newSecondaryLegState;
                toggle_secondary_leg_state_ = true;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::primaryTipVelocityInputCallback(const geometry_msgs::Point &input) {
    primary_tip_velocity_input_ = Eigen::Vector3d(input.x, input.y, input.z);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::secondaryTipVelocityInputCallback(const geometry_msgs::Point &input) {
    secondary_tip_velocity_input_ = Eigen::Vector3d(input.x, input.y, input.z);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::primaryTipPoseInputCallback(const geometry_msgs::Pose &input) {
    primary_pose_input_.position_ = Eigen::Vector3d(input.position.x, input.position.y, input.position.z);
    primary_pose_input_.rotation_ =
        Eigen::Quaterniond(input.orientation.w, input.orientation.x, input.orientation.y, input.orientation.z);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::secondaryTipPoseInputCallback(const geometry_msgs::Pose &input) {
    secondary_pose_input_.position_ = Eigen::Vector3d(input.position.x, input.position.y, input.position.z);
    secondary_pose_input_.rotation_ =
        Eigen::Quaterniond(input.orientation.w, input.orientation.x, input.orientation.y, input.orientation.z);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::parameterSelectionCallback(const int &input) {
    if (robot_state_ == RUNNING) {
        ParameterSelection new_parameter_selection = static_cast<ParameterSelection>(int(input));
        if (new_parameter_selection != parameter_selection_) {
            parameter_selection_ = new_parameter_selection;
            if (parameter_selection_ != NO_PARAMETER_SELECTION) {
                dynamic_parameter_ = params_.adjustable_map[parameter_selection_]; // Pointer to adjustable parameter object
                printf("\n%s parameter currently selected.\n", dynamic_parameter_->name.c_str());
            } else {
                printf("\nNo parameter currently selected.\n");
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::parameterAdjustCallback(const int &input) {
    if (robot_state_ == RUNNING) {
        int adjust_direction = input; // -1 || 0 || 1 (Decrease, no adjustment, increase)
        if (adjust_direction != 0.0 && !parameter_adjust_flag_ && parameter_selection_ != NO_PARAMETER_SELECTION) {
            double parameter_adjustment = dynamic_parameter_->adjust_step;
            if (sign(dynamic_parameter_->adjust_step) != sign(adjust_direction)) // If directions differ
            {
                parameter_adjustment *= -1; // Change direction
            }
            new_parameter_value_ = dynamic_parameter_->current_value + parameter_adjustment;
            new_parameter_value_ = clamped(new_parameter_value_,
                                           dynamic_parameter_->min_value,
                                           dynamic_parameter_->max_value);
            parameter_adjust_flag_ = true;
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::imuCallback(const sensor_msgs::Imu &data) {
    if (system_state_ != SUSPENDED && poser_ != NULL) {
        Eigen::Quaterniond orientation(data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z);
        Eigen::Vector3d angular_velocity(data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z);
        Eigen::Vector3d linear_acceleration(data.linear_acceleration.x,
                                            data.linear_acceleration.y,
                                            data.linear_acceleration.z);
        model_->setImuData(orientation, linear_acceleration, angular_velocity);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void StateController::jointStatesCallback(const sensor_msgs::JointState &joint_states) {
    bool get_effort_values = (joint_states.effort.size() != 0);
    bool get_velocity_values = (joint_states.velocity.size() != 0);

    // Iterate through message and assign found state values to joint objects
    for (uint i = 0; i < joint_states.name.size(); ++i) {
        std::string joint_name = joint_states.name[i];
        std::string leg_name = joint_name.substr(0, joint_name.find("_"));
        std::shared_ptr<Leg> leg = model_->getLegByIDName(leg_name);
        if (leg != NULL) {
            std::shared_ptr<Joint> joint = leg->getJointByIDName(joint_name);
            if (joint != NULL) {
                joint->current_position_ = joint_states.position[i] - joint->offset_;
                if (get_velocity_values) {
                    joint->current_velocity_ = joint_states.velocity[i];
                }
                if (get_effort_values) {
                    joint->current_effort_ = joint_states.effort[i];
                    joint->desired_effort_ = joint->current_effort_; // HACK
                }
            }
        }
    }

    // Check if all joint positions have been received from topic
    if (!joint_positions_initialised_) {
        joint_positions_initialised_ = true;
        for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_) {
            std::shared_ptr<Leg> leg = leg_it_->second;
            JointContainer::iterator joint_it;
            for (joint_it = leg->getJointContainer()->begin(); joint_it != leg->getJointContainer()->end(); ++joint_it) {
                std::shared_ptr<Joint> joint = joint_it->second;
                if (joint->current_position_ == UNASSIGNED_VALUE) {
                    joint_positions_initialised_ = false;
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::tipStatesCallback(const syropod_highlevel_controller::TipState &tip_states) {
    bool get_wrench_values = tip_states.wrench.size() > 0;
    bool get_step_plane_values = tip_states.step_plane.size() > 0;

    // Iterate through message and assign found contact proximity value to leg objects
    std::string error_string;
    for (uint i = 0; i < tip_states.name.size(); ++i) {
        std::string tip_name = tip_states.name[i];
        std::string leg_name = tip_name.substr(0, tip_name.find("_"));
        std::shared_ptr<Leg> leg = model_->getLegByIDName(leg_name);
        if (leg != NULL) {
            std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
            if (get_wrench_values) {
                Eigen::Vector3d tip_force(tip_states.wrench[i].force.x,
                                          tip_states.wrench[i].force.y,
                                          tip_states.wrench[i].force.z);
                Eigen::Vector3d tip_torque(tip_states.wrench[i].torque.x,
                                           tip_states.wrench[i].torque.y,
                                           tip_states.wrench[i].torque.z);
                if (leg_stepper != NULL) {
                    leg_stepper->setTouchdownDetection(true);
                }
                leg->setTipForceMeasured(tip_force);
                leg->setTipTorqueMeasured(tip_torque);
                leg->touchdownDetection();
            }
            if (get_step_plane_values) {
                if (leg_stepper != NULL) {
                    leg_stepper->setTouchdownDetection(true);
                }
                if (tip_states.step_plane[i].z != UNASSIGNED_VALUE) {
                    // From step plane representation calculate position and orientation of plane relative to tip frame
                    Eigen::Vector3d step_plane_position(tip_states.step_plane[i].z, 0.0, 0.0);
                    Eigen::Vector3d step_plane_normal(tip_states.step_plane[i].x, tip_states.step_plane[i].y, -1.0);
                    Eigen::Quaterniond step_plane_orientation = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d(0, 0, 1.0),
                                                                                                   -step_plane_normal);

                    // Transform into robot frame and store
                    Pose step_plane_pose = leg->getTip()->getPoseRobotFrame(Pose(step_plane_position, step_plane_orientation));
                    leg->setStepPlanePose(step_plane_pose);
                } else {
                    leg->setStepPlanePose(Pose::Undefined());
                    error_string += stringFormat("\nLost contact with tip range sensor/s of leg %s.\n", leg_name.c_str());
                }
            }
        }
    }
    if (!error_string.empty()) {
        printf("%s", error_string.c_str());
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::targetConfigurationCallback(const sensor_msgs::JointState &target_configuration) {
    poser_->setTargetConfiguration(target_configuration);
    target_configuration_acquired_ = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::targetBodyPoseCallback(const geometry_msgs::Pose &target_body_pose) {
    for (leg_it_ = model_->getLegContainer()->begin(); leg_it_ != model_->getLegContainer()->end(); ++leg_it_) {
        std::shared_ptr<Leg> leg = leg_it_->second;
        std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
        std::shared_ptr<LegPoser> leg_poser = leg->getLegPoser();
        leg_poser->setTargetTipPose(Pose::Undefined());
    }
    poser_->setTargetBodyPose(Pose(target_body_pose));
    target_body_pose_acquired_ = true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::targetTipPoseCallback(const syropod_highlevel_controller::TargetTipPose &msg) {
    if (robot_state_ == RUNNING) {
        for (uint i = 0; i < msg.name.size(); ++i) {
            std::shared_ptr<Leg> leg = model_->getLegByIDName(msg.name[i]);
            if (leg != NULL) {
                std::shared_ptr<LegStepper> leg_stepper = leg->getLegStepper();
                std::shared_ptr<LegPoser> leg_poser = leg->getLegPoser();

                if (leg->getIDName() == msg.name[i]) {
                    bool is_external_target = !msg.target.empty() && Pose(msg.target[i].pose) != Pose::Undefined();
                    bool is_external_default = !msg.stance.empty() && Pose(msg.stance[i].pose) != Pose::Undefined();

                    // Create external target from message and send to walk/pose controller depending on walk state
                    if (is_external_target) {
                        ExternalTarget external_target;
                        external_target.pose_ = Pose(msg.target[i].pose);
                        bool swing_clearance = msg.swing_clearance.size() > 0;
                        external_target.swing_clearance_ = static_cast<double>(swing_clearance ? msg.swing_clearance[i] : 0.0);

                        // external_target.frame_id_ = msg.target[i].header.frame_id;
                        external_target.transform_ = Pose::Identity(); // Correctly set from tf tree in main loop
                        external_target.defined_ = true;
                        if (walker_->getWalkState() != STOPPED) {
                            leg_stepper->setExternalTarget(external_target);
                        } else {
                            leg_poser->setExternalTarget(external_target);
                            target_tip_pose_acquired_ = true;
                        }
                    }

                    // Create external default from message and send to walk/ controller if walking
                    if (is_external_default && walker_->getWalkState() != STOPPED) {
                        ExternalTarget external_default;
                        external_default.pose_ = Pose(msg.stance[i].pose);
                        external_default.swing_clearance_ = 0.0;
                        // external_default.time_ = msg.stance[i].header.stamp;
                        // external_default.frame_id_ = msg.stance[i].header.frame_id;
                        external_default.transform_ = Pose::Identity(); // Correctly set from tf tree in main loop
                        external_default.defined_ = true;
                        leg_stepper->setExternalDefault(external_default);
                    }
                }
            } else {
                // Error
                printf("\nRequested target tip pose for leg '%s' failed. Leg '%s' does not exist in model.\n",
                       msg.name[i].c_str(), msg.name[i].c_str());
                return;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::initParameters(void) {
    // Control parameters
    params_.time_delta.init("time_delta");
    params_.imu_posing.init("imu_posing");
    params_.auto_posing.init("auto_posing");
    params_.rough_terrain_mode.init("rough_terrain_mode");
    params_.manual_posing.init("manual_posing");
    params_.inclination_posing.init("inclination_posing");
    params_.admittance_control.init("admittance_control");

    // Hardware interface parameters
    params_.individual_control_interface.init("individual_control_interface");
    params_.combined_control_interface.init("combined_control_interface");

    // Model parameters
    params_.syropod_type.init("syropod_type");
    params_.leg_id.init("leg_id");
    params_.joint_id.init("joint_id");
    params_.link_id.init("link_id");
    params_.leg_DOF.init("leg_DOF");
    params_.clamp_joint_positions.init("clamp_joint_positions");
    params_.clamp_joint_velocities.init("clamp_joint_velocities");
    params_.ignore_IK_warnings.init("ignore_IK_warnings");

    // Walk controller parameters
    params_.gait_type.init("gait_type");
    params_.body_clearance.init("body_clearance");
    params_.step_frequency.init("step_frequency");
    params_.swing_height.init("swing_height");
    params_.swing_width.init("swing_width");
    params_.step_depth.init("step_depth");
    params_.stance_span_modifier.init("stance_span_modifier");
    params_.velocity_input_mode.init("velocity_input_mode");
    params_.body_velocity_scaler.init("body_velocity_scaler");
    params_.force_cruise_velocity.init("force_cruise_velocity");
    params_.linear_cruise_velocity.init("linear_cruise_velocity");
    params_.angular_cruise_velocity.init("angular_cruise_velocity");
    params_.cruise_control_time_limit.init("cruise_control_time_limit");
    params_.overlapping_walkspaces.init("overlapping_walkspaces");
    params_.force_normal_touchdown.init("force_normal_touchdown");
    params_.gravity_aligned_tips.init("gravity_aligned_tips");
    params_.liftoff_threshold.init("liftoff_threshold");
    params_.touchdown_threshold.init("touchdown_threshold");

    // Pose controller parameters
    params_.auto_pose_type.init("auto_pose_type");
    params_.start_up_sequence.init("start_up_sequence");
    params_.time_to_start.init("time_to_start");
    params_.rotation_pid_gains.init("rotation_pid_gains");
    params_.max_translation.init("max_translation");
    params_.max_translation_velocity.init("max_translation_velocity");
    params_.max_rotation.init("max_rotation");
    params_.max_rotation_velocity.init("max_rotation_velocity");
    params_.leg_manipulation_mode.init("leg_manipulation_mode");

    // Admittance controller parameters
    params_.dynamic_stiffness.init("dynamic_stiffness");
    params_.use_joint_effort.init("use_joint_effort");
    params_.integrator_step_time.init("integrator_step_time");
    params_.virtual_mass.init("virtual_mass");
    params_.virtual_stiffness.init("virtual_stiffness");
    params_.load_stiffness_scaler.init("load_stiffness_scaler");
    params_.swing_stiffness_scaler.init("swing_stiffness_scaler");
    params_.virtual_damping_ratio.init("virtual_damping_ratio");
    params_.force_gain.init("force_gain");

    // Debug Parameters
    params_.debug_rviz.init("debug_rviz");
    params_.console_verbosity.init("console_verbosity");
    params_.debug_moveToJointPosition.init("debug_move_to_joint_position");
    params_.debug_stepToPosition.init("debug_step_to_position");
    params_.debug_swing_trajectory.init("debug_swing_trajectory");
    params_.debug_stance_trajectory.init("debug_stance_trajectory");
    params_.debug_execute_sequence.init("debug_execute_sequence");
    params_.debug_workspace_calc.init("debug_workspace_calculations");
    params_.debug_IK.init("debug_ik");

    // Init all joint and link parameters per leg
    if (params_.leg_id.initialised && params_.joint_id.initialised && params_.link_id.initialised) {
        std::vector<std::string>::iterator leg_name_it;
        std::vector<std::string> leg_ids = params_.leg_id.data;
        int leg_id_num = 0;
        for (leg_name_it = leg_ids.begin(); leg_name_it != leg_ids.end(); ++leg_name_it, ++leg_id_num) {
            std::string leg_id_name = *leg_name_it;
            params_.leg_stance_positions[leg_id_num].init(leg_id_name + "_stance_position");
            params_.link_parameters[leg_id_num][0].init(leg_id_name + "_base_link_parameters");
            uint joint_count = params_.leg_DOF.data[leg_id_name];

            if (joint_count > params_.joint_id.data.size() || joint_count > params_.link_id.data.size() + 1) {
                printf("\nModel initialisation error for leg %s: Insufficient joint or link id's for defined DOF (%d).\n",
                       leg_id_name.c_str(), joint_count);

            } else {
                for (uint i = 1; i < joint_count + 1; ++i) {
                    std::string link_name = params_.link_id.data[i];
                    std::string joint_name = params_.joint_id.data[i - 1];
                    std::string link_parameter_name = leg_id_name + "_" + link_name + "_link_parameters";
                    std::string joint_parameter_name = leg_id_name + "_" + joint_name + "_joint_parameters";
                    params_.link_parameters[leg_id_num][i].init(link_parameter_name);
                    params_.joint_parameters[leg_id_num][i - 1].init(joint_parameter_name);
                }
            }
        }
    }

    // Generate adjustable parameter map for parameter adjustment selection
    params_.adjustable_map.insert(AdjustableMapType::value_type(STEP_FREQUENCY, &params_.step_frequency));
    params_.adjustable_map.insert(AdjustableMapType::value_type(SWING_HEIGHT, &params_.swing_height));
    params_.adjustable_map.insert(AdjustableMapType::value_type(SWING_WIDTH, &params_.swing_width));
    params_.adjustable_map.insert(AdjustableMapType::value_type(STEP_DEPTH, &params_.step_depth));
    params_.adjustable_map.insert(AdjustableMapType::value_type(STANCE_SPAN_MODIFIER, &params_.stance_span_modifier));
    params_.adjustable_map.insert(AdjustableMapType::value_type(VIRTUAL_MASS, &params_.virtual_mass));
    params_.adjustable_map.insert(AdjustableMapType::value_type(VIRTUAL_STIFFNESS, &params_.virtual_stiffness));
    params_.adjustable_map.insert(AdjustableMapType::value_type(VIRTUAL_DAMPING, &params_.virtual_damping_ratio));
    params_.adjustable_map.insert(AdjustableMapType::value_type(FORCE_GAIN, &params_.force_gain));

    initGaitParameters(GAIT_UNDESIGNATED);
    initAutoPoseParameters();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::initGaitParameters(const GaitDesignation &gait_selection) {
    switch (gait_selection) {
    case (TRIPOD_GAIT):
        params_.gait_type.data = "tripod_gait";
        break;
    case (RIPPLE_GAIT):
        params_.gait_type.data = "ripple_gait";
        break;
    case (WAVE_GAIT):
        params_.gait_type.data = "wave_gait";
        break;
    case (AMBLE_GAIT):
        params_.gait_type.data = "amble_gait";
        break;
    case (GAIT_UNDESIGNATED):
        params_.gait_type.init("gait_type");
        break;
    default:
        break;
    }

    std::string base_gait_parameters_name = "syropod/gait_parameters/";
    params_.stance_phase.init("stance_phase", base_gait_parameters_name + params_.gait_type.data + "/");
    params_.swing_phase.init("swing_phase", base_gait_parameters_name + params_.gait_type.data + "/");
    params_.phase_offset.init("phase_offset", base_gait_parameters_name + params_.gait_type.data + "/");
    params_.offset_multiplier.init("offset_multiplier", base_gait_parameters_name + params_.gait_type.data + "/");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void StateController::initAutoPoseParameters(void) {
    std::string base_auto_pose_parameters_name = "syropod/auto_pose_parameters/";
    if (params_.auto_pose_type.data == "auto") {
        base_auto_pose_parameters_name += (params_.gait_type.data + "_pose/");
    } else {
        base_auto_pose_parameters_name += (params_.auto_pose_type.data + "/");
    }

    params_.pose_frequency.init("pose_frequency", base_auto_pose_parameters_name);
    params_.pose_phase_length.init("pose_phase_length", base_auto_pose_parameters_name);
    params_.pose_phase_starts.init("pose_phase_starts", base_auto_pose_parameters_name);
    params_.pose_phase_ends.init("pose_phase_ends", base_auto_pose_parameters_name);
    params_.pose_negation_phase_starts.init("pose_negation_phase_starts", base_auto_pose_parameters_name);
    params_.pose_negation_phase_ends.init("pose_negation_phase_ends", base_auto_pose_parameters_name);
    params_.negation_transition_ratio.init("negation_transition_ratio", base_auto_pose_parameters_name);
    params_.x_amplitudes.init("x_amplitudes", base_auto_pose_parameters_name);
    params_.y_amplitudes.init("y_amplitudes", base_auto_pose_parameters_name);
    params_.z_amplitudes.init("z_amplitudes", base_auto_pose_parameters_name);
    params_.gravity_amplitudes.init("gravity_amplitudes", base_auto_pose_parameters_name);
    params_.roll_amplitudes.init("roll_amplitudes", base_auto_pose_parameters_name);
    params_.pitch_amplitudes.init("pitch_amplitudes", base_auto_pose_parameters_name);
    params_.yaw_amplitudes.init("yaw_amplitudes", base_auto_pose_parameters_name);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
