/***********************************************************************/
/**                                                                    */
/** HuNavSystemPlugin_fortress.cpp                                     */
/**                                                                    */
/** Copyright (c) 2025, Service Robotics Lab (SRL).                    */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Noé Pérez-Higueras (maintainer)                                    */
/** email: noeperez@upo.es                                             */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the MIT license. See the LICENSE file for details.              */
/**                                                                    */
/**                                                                    */
/***********************************************************************/

// #include <functional>
#include <stdio.h>
#include <string>
#include <chrono>
#include <stdexcept>
#include <sstream>
#include <cmath>

#include <hunav_gazebo_fortress_wrapper/HuNavSystemPlugin_fortress.h>
#include <gz/plugin/Register.hh>
#include <gz/plugin/RegisterMore.hh>
#include <gz/sim/Events.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/System.hh>


#define DEF_WALKING_ANIMATION "walk"

// IMPORTANT:
// WE MUST USE ONLY ACTORS LEAD BY HUNAVSIM,
// NOT THE ONES LEAD BY GAZEBO WALKING ANIMATIONS
// SINCE THE LATTER DO NOT HAVE PHYSICAL DYNAMICS
// AND DO NOT GENERATE WORLD POSITION AND VELOCITIES
// IN THE ENTITY COMPONENT MANAGER (ECM)



// The error occurs because Ignition Gazebo's `EntityComponentManager` uses a custom traits system to check for equality, and this conflicts with the standard `std::chrono::duration` equality operator. This results in an ambiguous overload for the `operator==`.
//To resolve this issue, you can explicitly specialize the `HasEqualityOperator` trait for `std::chrono::steady_clock::duration` to indicate that it has a valid equality operator. Add the following code at the top of your source file (before any other code):
namespace ignition::gazebo::v6::traits {
  template<>
  struct HasEqualityOperator<std::chrono::steady_clock::duration> {
    static constexpr bool value = true;
  };
}


using namespace std::chrono_literals;

/////////////////////////////////////////////////
HuNavSystemPluginIGN::HuNavSystemPluginIGN()
{
}

HuNavSystemPluginIGN::~HuNavSystemPluginIGN()
{
}

/////////////////////////////////////////////////
void HuNavSystemPluginIGN::Configure(const gz::sim::Entity& _entity, const std::shared_ptr<const sdf::Element>& _sdf,
                            gz::sim::EntityComponentManager& _ecm, gz::sim::EventManager& _eventMgr)
{
  
  (void)_eventMgr;
  counter_ = 0;

  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
  std::string nodename = "hunav_plugin_node_" + std::to_string(_entity);
  this->rosnode_ = std::make_shared<rclcpp::Node>(nodename.c_str());
  //this->ros_test_pub_ = this->rosnode_->create_publisher<std_msgs::msg::String>("/gz/ros/paused", 10);

  worldEntity_ = _entity; 

  // Print the number of entities in the world
  auto size = _ecm.EntityCount();
  ignmsg << "Number of entities: " << size << std::endl;
  ignmsg << "Number of this entity: " << _entity << std::endl;


  // auto entity_table = _ecm.EntityByComponents(gz::sim::components::Name("table2"));
  // ignmsg << "'table2' entity: " << entity_table << std::endl;

  // Comprobar si la entidad tiene un nombre
  auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity);
  if (nameComp)
  {
    //std::cout << "Plugin cargado en la entidad: " << nameComp->Data() << std::endl;
    ignmsg << "Plugin loaded in entity: " << nameComp->Data() << std::endl;
  }

  // Get the plugin parameters
  //----------------------------------------------------------------
  sdf_ = _sdf->Clone();

  // Robot model name in Gazebo
  if (sdf_->HasElement("robot_name"))
    robotName_ = sdf_->Get<std::string>("robot_name");
  else
    robotName_ = "robot";


  // frame to publish the position of the agents
  if (sdf_->HasElement("global_frame_to_publish"))
    globalFrame_ = sdf_->Get<std::string>("global_frame_to_publish");
  else
    globalFrame_ = "map";

  if (sdf_->HasElement("use_navgoal_to_start"))
    waitForGoal_ = sdf_->Get<bool>("use_navgoal_to_start");
  else
  {
    waitForGoal_ = false;
    //RCLCPP_INFO(this->rosnode_->get_logger(), "PARAMETER USE_NAVGOAL_TO_START IS NOT IN THE WORLD FILE!!");
    ignwarn << "PARAMETER USE_NAVGOAL_TO_START IS NOT IN THE WORLD FILE!!" << std::endl;
  }

  if (waitForGoal_)
  {
    goalReceived_ = false;
    if (sdf_->HasElement("navgoal_topic"))
      goalTopic_ = sdf_->Get<std::string>("navgoal_topic");
    else
      goalTopic_ = "goal_pose";
  }
  else
  {
    goalReceived_ = true;
  }

  // Read models to be ignored
  if (sdf_->HasElement("ignore_models"))
  {
    sdf::ElementPtr modelElem = sdf_->GetElement("ignore_models")->GetElement("model");
    while (modelElem)
    {
      ignoreModels_.push_back(modelElem->Get<std::string>());
      //RCLCPP_INFO(this->rosnode_->get_logger(), "Ignoring model: %s", (modelElem->Get<std::string>()).c_str());
      ignmsg << "Ignoring model: " << (modelElem->Get<std::string>()) << std::endl;
      modelElem = modelElem->GetNextElement("model");
    }
  }
  reset_ = false;
  //rostime_ = this->rosnode_->get_clock()->now();
  //dt_ = 0;
  // Update rate
  auto update_rate = sdf_->Get<double>("update_rate", 1000.0).first;
  if (update_rate > 0.0)
  {
    update_rate_secs_ = 1.0 / update_rate;
  }
  else
  {
    update_rate_secs_ = 0.0;
  }
  //RCLCPP_INFO(this->rosnode_->get_logger(), "update_rate: %.2f, secs:%.4f", update_rate, update_rate_secs_);
  ignmsg << "update_rate: " << update_rate << ", secs: " << update_rate_secs_ << std::endl;

  // Show all the entities in the world
  _ecm.Each<gz::sim::components::Name>([&](const gz::sim::Entity& _entity, const gz::sim::components::Name* _name)
  {
    ignmsg << "Entity: " << _entity << " Name: " << _name->Data() << std::endl;
    return true;
  });

  agentsInitialized_ = false;

  // How to get models and actors
  // auto model = gz::sim::Model(_entity);
  // model.SetWorldPoseCmd(EntityComponentManager &_ecm, const math::Pose3d &_pose)
  // auto actor = model.ActorByName("actor3");

  // service to initialize the agents from the WorldGenerator
  rosSrvGetAgentsClient_ = this->rosnode_->create_client<hunav_msgs::srv::GetAgents>("get_agents");
  //rosSrvGetAgentsClient_ = this->rosnode_->create_client<hunav_msgs::srv::GetAgent>("get_agent");
  // regular HuNavSim service to compute the agents
  rosSrvClient_ = this->rosnode_->create_client<hunav_msgs::srv::ComputeAgents>("compute_agents");
  //rosSrvClient_ = this->rosnode_->create_client<hunav_msgs::srv::MoveAgent>("move_agent");

  rosSrvResetClient_ = this->rosnode_->create_client<hunav_msgs::srv::ResetAgents>("reset_agents");

  // Initiate the agents and the robot
  try
  {
    initializeAgents(_ecm);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->rosnode_->get_logger(), "Error initializing agents: %s. Exiting...", e.what());
    ignerr << "Error initializing agents. Exiting..." << std::endl;
    rclcpp::shutdown();
    return;
  }
  try
  {
    getObstacles(_ecm); 
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->rosnode_->get_logger(), "Error handling obstacles: %s. Exiting...", e.what());
    ignerr << "Error handling obstacles: " << e.what() << ". Exiting..." << std::endl;
    rclcpp::shutdown();
    return;
  }
  ignmsg << "Plugin HuNavPluginIGN configured!!!!!!!!!!!!!!!" << std::endl;
}





/////////////////////////////////////////////////
/**
 * @brief initialize the robot status
 * @param _ecm EntityComponentManager
 */
void HuNavSystemPluginIGN::initializeRobot(gz::sim::EntityComponentManager& _ecm)
{
  //RCLCPP_INFO(this->rosnode_->get_logger(), "Initializing robot...");
  ignmsg << "Initializing robot..." << std::endl;

  robotEntity_ = _ecm.EntityByComponents(gz::sim::components::Name(robotName_));
  if(robotEntity_) {
    //RCLCPP_INFO_STREAM(this->rosnode_->get_logger(), "Robot " << robotName_ << " found!" << std::endl);
    ignmsg << "Robot " << robotName_ << " found!" << std::endl;
  }
  else {
    //RCLCPP_ERROR(this->rosnode_->get_logger(), "Robot model %s not found (yet)!!!!", robotName_.c_str());
    ignwarn << "Robot model " << robotName_ << " not found (yet)!!!!" << std::endl;
    throw std::runtime_error("Error initializing agents. Robot model not found. Exiting...");
  }

  robotLastTime_ = std::chrono::steady_clock::now();
  rosRobotLastTime_ = this->rosnode_->get_clock()->now();

  gz::math::Pose3d pose = worldPose(robotEntity_, _ecm);
  robotAgent_.id = robotEntity_;
  robotAgent_.type = hunav_msgs::msg::Agent::ROBOT;
  robotAgent_.behavior.type = hunav_msgs::msg::AgentBehavior::BEH_REGULAR;
  robotAgent_.behavior.state = hunav_msgs::msg::AgentBehavior::BEH_NO_ACTIVE;
  robotAgent_.name = robotName_;
  robotAgent_.group_id = -1;
  robotAgent_.radius = 0.35;
  robotAgent_.position.position.x = pose.Pos().X();
  robotAgent_.position.position.y = pose.Pos().Y();
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, pose.Rot().Z());
  robotAgent_.position.orientation = tf2::toMsg(myQuaternion);
  robotAgent_.yaw = pose.Rot().Z();
  // auto linv = _ecm.Component<gz::sim::components::LinearVelocity>(robotEntity_);
  // gz::math::Vector3d linvel = linv->Data();
  robotAgent_.velocity.linear.x = 0.0;  //linvel.X();
  robotAgent_.velocity.linear.y = 0.0; //linvel.Y();
  robotAgent_.linear_vel = 0.0; //linvel.Length();
  // auto angv = _ecm.Component<gz::sim::components::AngularVelocity>(robotEntity_);
  // gz::math::Vector3d angvel = angv->Data();
  robotAgent_.velocity.angular.z = 0.0; //angvel.Z();
  robotAgent_.angular_vel = 0.0; //angvel.Z();
  initRobotAgent_ = robotAgent_;
  //RCLCPP_INFO_STREAM(this->rosnode_->get_logger(), "Robot pos x: " << robotAgent_.position.position.x << ", y: " << robotAgent_.position.position.y << ", vel: " << robotAgent_.linear_vel);
  //ignmsg << "Robot pos x: " << robotAgent_.position.position.x << ", y: " << robotAgent_.position.position.y << std::endl; // << ", vel: " << robotAgent_.linear_vel << std::endl;
}




/////////////////////////////////////////////////ç
/**
 * @brief Initialize the agents status from the WorldGenerator
 * @param _ecm EntityComponentManager
 */
void HuNavSystemPluginIGN::initializeAgents(gz::sim::EntityComponentManager& _ecm)
{
  //RCLCPP_INFO(rosnode->get_logger(), "Initializing agents...");
  ignmsg << "Initializing agents..." << std::endl;

  // initiate the robot state
  try
  {
    initializeRobot(_ecm);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(this->rosnode_->get_logger(), "Error initializing robot. We will try again...");
    //ignerr << "Error initializing robot. Exiting..." << std::endl;
    //rclcpp::shutdown();
    return;
  }

  // Fill the service request
  auto request = std::make_shared<hunav_msgs::srv::GetAgents::Request>();
  request->empty = 0;

  // Wait for the service to be available
  while (!rosSrvGetAgentsClient_->wait_for_service(5s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rosnode_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      throw std::runtime_error("Interrupted while waiting for the service /get_agents");
    }
    RCLCPP_WARN(rosnode_->get_logger(), "service /get_agents not available, waiting again...");
  }
  RCLCPP_INFO(rosnode_->get_logger(), "Service /get_agents is available. Calling service...");
  // Call the service
  auto result = rosSrvGetAgentsClient_->async_send_request(request);
  //std::chrono::duration<int, std::milli> ms(4000);
  //if (result.wait_for(ms) == std::future_status::ready)
  if (rclcpp::spin_until_future_complete(this->rosnode_, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    // Initialize the actors
    auto res = *result.get();
    const hunav_msgs::msg::Agents agents = res.agents;
    RCLCPP_INFO(this->rosnode_->get_logger(), "Received %i agents from service /get_agents", (int)agents.agents.size());
    pedestrians_.clear();
    pedLastTime_ = std::chrono::steady_clock::now();
    rosPedLastTime_ = this->rosnode_->get_clock()->now();

    for (const auto &agent : agents.agents)
    {
      hunav_msgs::msg::Agent ag = agent;
      auto agentEntity = _ecm.EntityByComponents(gz::sim::components::Name(agent.name));
      if(!agentEntity) {
        //RCLCPP_ERROR(rosnode->get_logger(), "Pedestrian model '%s' not found!!!!", agent.name.c_str());
        ignerr << "ERROR! Pedestrian model " << agent.name << " not found!" << std::endl;
        // std::ostringstream oss;
        // oss << "ERROR: Pedestrian model " << agent.name << " not found!" << std::endl;
        // throw std::runtime_error(oss.str());
        agentsInitialized_ = false;
        break;
      }

      //Actor
      auto actorComp = _ecm.Component<gz::sim::components::Actor>(agentEntity);
      if (!actorComp)
      {
        ignerr << "Entity [" << agentEntity << "] is not an actor." << std::endl;
        return;
      }
      if (actorComp->Data().AnimationCount() < 1)
      {
        ignerr << "Actor [" << actorComp->Data().Name()  << "] SDF doesn't have any animations." << std::endl;
        return;
      }
      ignmsg << "Actor [" << actorComp->Data().Name()  << "] has " << actorComp->Data().AnimationCount() << " animations" << std::endl;
      // we take and apply the first animation!!!!
      auto ani = actorComp->Data().AnimationByIndex(0);
      ignmsg << "Animation name: " << ani->Name() << std::endl;
      ignmsg << "Animation filename: " << ani->Filename() << std::endl;

      // Animation name
      auto animNameComp = _ecm.Component<gz::sim::components::AnimationName>(agentEntity);
      if(!animNameComp)
      {
        ignwarn << "AnimationName component does not exist. Creating..." << std::endl;
        _ecm.SetComponentData<gz::sim::components::AnimationName>(agentEntity, ani->Name().c_str()); //DEF_WALKING_ANIMATION); //ani->Name().c_str());
        //_ecm.SetChanged(entity, gz::sim::components::AnimationName::typeId, gz::sim::ComponentState::OneTimeChange);
      }
      else
      {
        *animNameComp = gz::sim::components::AnimationName(ani->Name().c_str()); //DEF_WALKING_ANIMATION); //ani->Name().c_str());
        ignmsg << "Actor [" << actorComp->Data().Name()  << "] has animation name: " << animNameComp->Data() << std::endl;
      }
      _ecm.SetChanged(agentEntity, gz::sim::components::AnimationName::typeId, gz::sim::ComponentState::OneTimeChange);

      // Animation time
      auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(agentEntity);
      if (!animTimeComp)
      {
        ignwarn << "AnimationTime component does not exist. Creating..." << std::endl;
        //_ecm.CreateComponent(entity, gz::sim::components::AnimationTime());
        std::chrono::steady_clock::duration oneSecond = std::chrono::seconds(0);
        _ecm.SetComponentData<gz::sim::components::AnimationTime>(agentEntity, oneSecond);
        //_ecm.SetComponentData<gz::sim::components::AnimationTime>(entity, gz::sim::components::AnimationTime());
        //_ecm.SetComponentData<gz::sim::components::AnimationTime>(entity, std::chrono::duration_cast<std::chrono::seconds>(1s));
        //_ecm.SetChanged(entity, gz::sim::components::AnimationTime::typeId, gz::sim::ComponentState::OneTimeChange); 
      }
      else
      {
        auto animTime = animTimeComp->Data();
        double animTimeInSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(animTime).count();
        ignmsg << "Actor [" << actorComp->Data().Name()  << "] has animation time: " << animTimeInSeconds << std::endl;
      }


      //auto curr_pose = worldPose(agentEntity, _ecm);
      //ignmsg << "Initialize. Actor [" << agent.name << "] pose: " << curr_pose << std::endl;
      gz::math::Pose3d curr_pose;
      curr_pose.Pos().X(agent.position.position.x);
      curr_pose.Pos().Y(agent.position.position.y);
      curr_pose.Pos().Z(0.35);
      curr_pose.Rot() = gz::math::Quaterniond(0, 0, ag.yaw);

      // we try to give a name to the trajectory that matches the animation name
      // BUT IT DOES NOT WORK!!!!
      // sdf::Trajectory traj;
      // traj.SetType(ani->Name());
      // traj.SetId(1);
      // sdf::Waypoint wp;
      // wp.SetPose(curr_pose);
      // traj.AddWaypoint(wp);
      // actorComp->Data().AddTrajectory(traj);
      // _ecm.SetChanged(agentEntity, gz::sim::components::Actor::typeId, gz::sim::ComponentState::OneTimeChange);



      // We'll be setting the actor's X/Y pose with respect to the world. So we
      // zero the current values.
      auto lposeComp = _ecm.Component<gz::sim::components::Pose>(agentEntity);
      if (nullptr == lposeComp)
      {
        _ecm.CreateComponent(agentEntity, gz::sim::components::Pose(
            gz::math::Pose3d::Zero));
        //_ecm.SetComponentData<gz::sim::components::Pose>(agentEntity, curr_pose);
      }
      else
      {
        auto p = lposeComp->Data();
        auto newPose = p;
        newPose.Pos().X(0);
        newPose.Pos().Y(0);
        *lposeComp = gz::sim::components::Pose(newPose);
        //lposeComp->Data() = curr_pose;
      }
      // setChanged necessary here???
      _ecm.SetChanged(agentEntity, gz::sim::components::Pose::typeId, gz::sim::ComponentState::OneTimeChange);


      auto wpose = _ecm.Component<gz::sim::components::WorldPose>(agentEntity);
      if(wpose)
      {
        wpose->Data() = curr_pose;
      }
      else
      {
        //By default,this happens. The WorldPose component does not exist. So, we create it.
        ignwarn << "WARNING! Could not get World pose of actor " << agent.name << ". Creating component..." << std::endl;
        _ecm.SetComponentData<gz::sim::components::WorldPose>(agentEntity, curr_pose);
        //_ecm.SetComponentData<gz::sim::components::Pose>(agentEntity, curr_pose);
      }
      _ecm.SetChanged(agentEntity, gz::sim::components::WorldPose::typeId, gz::sim::ComponentState::OneTimeChange);
      // // //-----------------------------------




      // Trajectory
      //auto trajCount = actorComp->Data().TrajectoryCount();
      //ignmsg << "Actor [" << actorComp->Data().Name()  << "] has " << trajCount << " trajectories" << std::endl;
      // Having a trajectory pose prevents the actor from moving with the
      // SDF script
      auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(agentEntity);
      if (nullptr == trajPoseComp)
      {
        ignwarn << "TrajectoryPose component does not exist. Creating..." << std::endl;
        
        //_ecm.CreateComponent(entity, gz::sim::components::TrajectoryPose(initialPose));
        // Aquí probar si pasar la pose o la WorldPose. 
        _ecm.SetComponentData<gz::sim::components::TrajectoryPose>(agentEntity, curr_pose);
        //_ecm.SetChanged(entity, gz::sim::components::AnimationName::typeId, gz::sim::ComponentState::OneTimeChange);
      }
      else
      {
        *trajPoseComp = gz::sim::components::TrajectoryPose(curr_pose);
      }
      _ecm.SetChanged(agentEntity, gz::sim::components::TrajectoryPose::typeId, gz::sim::ComponentState::OneTimeChange);


      // check the pose of the actor has been updated
      auto npose = worldPose(agentEntity, _ecm);
      // auto trajec = _ecm.Component<gz::sim::components::TrajectoryPose>(agentEntity);
      // ignmsg << "Actor [" << agent.name << "] new pose: " << npose << std::endl;
      // ignmsg << "Actor [" << agent.name << "] trajectory pose: " << trajec->Data() << std::endl;
      

      double yaw = normalizeAngle(npose.Rot().Yaw());
      ag.position.position.x = npose.Pos().X();
      ag.position.position.y = npose.Pos().Y();
      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, yaw);
      ag.position.orientation = tf2::toMsg(myQuaternion);
      ag.yaw = yaw;
      // ignition::math::Vector3d linvel = model->WorldLinearVel();
      ag.desired_velocity = agent.desired_velocity;
      ag.velocity.linear.x = 0.0; //linvel.X();
      ag.velocity.linear.y = 0.0; //linvel.Y();
      ag.linear_vel = 0.0; //linvel.Length();
      // ignition::math::Vector3d angvel = model->WorldAngularVel();
      ag.velocity.angular.z = 0.0;  //angvel.Z();
      ag.angular_vel = 0.0; // angvel.Z();

      pedestrians_[agentEntity] = ag;
      RCLCPP_INFO(rosnode_->get_logger(),
                  "Properties of agent %s initialized", ag.name.c_str());
    }
  }
  else
  {
    RCLCPP_ERROR(rosnode_->get_logger(), "Failed to call service get_agents");
    throw std::runtime_error("ERROR: Failed to call service get_agents");
  }

  ///////// OLD CODE ///////////////////////////////////////////////////////////7
  // hnav_->lastUpdate = _world->SimTime();
  // // Reset trajectory so we can move the agents!
  // for (const auto& [entity, ped] : pedestrians_)
  // {
  //   //gazebo::physics::ModelPtr model = hnav_->world->ModelByName(hnav_->pedestrians[i].name);
  //   //gazebo::physics::ActorPtr actor = boost::dynamic_pointer_cast<gazebo::physics::Actor>(model);
  //   // Retrieve the Actor component
  //   auto actorComp = _ecm.Component<gz::sim::components::Actor>(entity);
  //   if (actorComp)
  //   {
  //     auto actor = gz::sim::Actor(entity);
  //     //auto actor = actorComp->Data();
  //     auto actorName = actor.Name(_ecm);
  //     auto animName = actor.AnimationName(_ecm);
  //     //auto animSpeed = actor.AnimationSpeed(_ecm);
  //     auto animTime = actor.AnimationTime(_ecm);
  //     if(actorName)
  //       ignmsg << "Actor name: " << actorName.value() << std::endl; // ", Time: " << animTime.value() << std::endl;
  //     if(animName)
  //       ignmsg << "Animation name: " << animName.value() << std::endl;
  //     // // Create an instance of AnimationUpdateData
  //     // gz::sim::components::AnimationUpdateData animationUpdateData;
  //     // // Set the desired animation parameters
  //     // animationUpdateData.SetAnimationName("no_active");
  //     // //animationUpdateData.SetAnimationSpeed(animationSpeed);
  //     // //animationUpdateData.trajectory.duration = 1.0;
  //     // animationUpdateData.trajectory.SetId(0);
  //     // // Apply the animation update to the Actor component
  //     // _ecm.SetComponentData<gz::sim::components::AnimationUpdateData>(entity, animationUpdateData);
  //     // _ecm.SetChanged(entity, gz::sim::components::AnimationUpdateData::typeId);
  //         // Update the animation name
  //     //_ecm.SetComponentData<gz::sim::components::AnimationName>(entity, "no_active");
  //     //_ecm.SetChanged(entity, gz::sim::components::AnimationName::typeId);

  //   // Update the animation time
  //   //_ecm.SetComponentData<gz::sim::components::AnimationTime>(entity, std::chrono::steady_clock::duration::zero());
  //   //_ecm.SetChanged(entity, gz::sim::components::AnimationTime::typeId);
  //   }
  //   else
  //   {
  //     ignerr << "InitAgent " << ped.name << ". Actor entity does not have an Actor component" << std::endl;
  //   }

    // gazebo::physics::TrajectoryInfoPtr trajectoryInfo;
    // trajectoryInfo.reset(new gazebo::physics::TrajectoryInfo());
    // trajectoryInfo->id = 0;
    // auto skelAnims = actor->SkeletonAnimations();
    // if (!skelAnims.empty())
    // {
    //   for (auto it = skelAnims.begin(); it != skelAnims.end(); ++it)
    //   {
    //     RCLCPP_INFO(hnav_->rosnode->get_logger(), "Agent %s has animation: %s", actor->GetName().c_str(),
    //                 it->first.c_str());
    //   }
    //   trajectoryInfo->type = "no_active";  // skelAnims.begin()->first;
    // }
    // else
    // {
    //   RCLCPP_INFO(hnav_->rosnode->get_logger(), "Animation default type: %s", DEF_WALKING_ANIMATION);
    //   trajectoryInfo->type = DEF_WALKING_ANIMATION;
    // }
    // trajectoryInfo->duration = 1.0;

    // /// \brief Set a custom trajectory for the actor, using one of the
    // /// existing animations. This will override any trajectories previously
    // /// defined. When a custom trajectory is defined, the script time must
    // /// be set with `SetScriptTime` in order to play the animation.
    // /// \param[in] _trajInfo Information about custom trajectory.
    // /// \sa ResetCustomTrajectory, SetScriptTime
    // /// public: void SetCustomTrajectory(TrajectoryInfoPtr &_trajInfo);
    // actor->SetCustomTrajectory(trajectoryInfo);
  // }
  //////////////////////////////////////////////////////////////////////////////////////////////77
  agentsInitialized_ = true;

}






/////////////////////////////////////////////////
/**
 * @brief Get the closest obstacle for each pedestrian
 * @param _ecm EntityComponentManager
 */
void HuNavSystemPluginIGN::getObstacles(const gz::sim::EntityComponentManager& _ecm)
{

  //for(auto const& [agentEntity, agent] : pedestrians_)
  for (const auto& p : pedestrians_)
  {
    double minDist = 5.0; //10000.0;
    //ignition::math::Vector3d closest_obstacle;
    // ignition::math::Vector3d closest_obs2;
    pedestrians_[p.first].closest_obs.clear();

    //gz::math::Pose3d actor_pose = worldPose(p.first, _ecm);
    //gz::math::Pose3d actor_pose = _ecm.Component<gz::sim::components::Pose>(p.first)->Data();
    auto traj = _ecm.Component<gz::sim::components::TrajectoryPose>(p.first);
    if(!traj)
    {
      ignerr << "ERROR! Actor " << p.first << " does not have a TrajectoryPose component" << std::endl;
      continue;
    }
    auto actor_pose = traj->Data();

    // we create a default bounding box for the actor
    auto actor_size = gz::math::Vector3d(0.35, 0.35, 1.65); 
    gz::math::AxisAlignedBox actor_bb = gz::math::AxisAlignedBox(
      actor_pose.Pos() - actor_size / 2,
      actor_pose.Pos() + actor_size / 2
    );
    //ignmsg << std::endl << "Checking obstacles entities for Actor " << p.first << ", name: " << actor_name->Data() << std::endl;
    //ignmsg << "Actor pose: " << actor_pose << std::endl;

    // for each actor, iterate over all the models in the world
    std::unordered_map<gz::sim::Entity, gz::math::Pose3d> obs_entities;
    // POSE OR WORLDPOSE?
    _ecm.Each<gz::sim::components::Pose>([&](const gz::sim::Entity& _entity, const gz::sim::components::Pose* _enty_pose)
    {
      (void)_enty_pose;
      auto enty_name = _ecm.Component<gz::sim::components::Name>(_entity);
      // Do not consider yourserf and the models to be ignored
      if(_entity != p.first && std::find(ignoreModels_.begin(), ignoreModels_.end(), enty_name->Data()) == ignoreModels_.end())
      {
        //WorldPose instead of Pose
        auto wp = worldPose(_entity, _ecm);
        obs_entities[_entity] = wp; // _enty_pose->Data(); //wp;
      }
      return true;
    });  

    // Iterate over the obstacles and compute the distance to the actor
    for (const auto& [obs_enty, obs_pose] : obs_entities)
    {
      auto obs_name = _ecm.Component<gz::sim::components::Name>(obs_enty);
      if(firstObstaclePrint_)
        RCLCPP_INFO_STREAM(this->rosnode_->get_logger(), "Entity " << obs_enty <<  " Obstacle name: " << obs_name->Data() << ", position [" << obs_pose.Pos().X() << "," << obs_pose.Pos().Y() << "]" << std::endl);
      //gz::math::Vector3d offset = obs_pose.Pos() - actor_pose.Pos();
      //double modelDist = offset.Length();
      // ignmsg << std::endl;
      // ignmsg << "Entity: " << obs_name->Data() << " Pose: " << obs_pose.Pos() << std::endl;
      // ignmsg << "Actor: " << actor_name->Data() << " Pose: " << actor_pose.Pos() << std::endl; //pose->Data() is a gz::math::Pose3d
      // ignmsg << "Distance between " << obs_name->Data() << " and actor " << actor_name->Data() << " is " << modelDist << " m" << std::endl;
      
      // ALIGNEDBOXES CAN NOT BE ACCESSED
      // auto act_alignedBox = _ecm.Component<gz::sim::components::AxisAlignedBox>(obs_enty);
      // This is allways nullptr
      
      // we check if the entity has a geometry component and we build the bounding box
      auto geometry = _ecm.Component<gz::sim::components::Geometry>(obs_enty);
      if (geometry)
      {
        gz::math::AxisAlignedBox obs_bb;
        if (geometry->Data().Type() == sdf::GeometryType::BOX)
        {
          auto box = geometry->Data().BoxShape();
          obs_bb = gz::math::AxisAlignedBox(
            obs_pose.Pos() - box->Size() / 2,
            obs_pose.Pos() + box->Size() / 2
          );
        }
        else if (geometry->Data().Type() == sdf::GeometryType::SPHERE)
        {
          auto sphere = geometry->Data().SphereShape();
          double radius = sphere->Radius();
          obs_bb = gz::math::AxisAlignedBox(
            obs_pose.Pos() - gz::math::Vector3d(radius, radius, radius),
            obs_pose.Pos() + gz::math::Vector3d(radius, radius, radius)
          );
        }
        else if (geometry->Data().Type() == sdf::GeometryType::CYLINDER)
        {
          auto cylinder = geometry->Data().CylinderShape();
          double radius = cylinder->Radius();
          double height = cylinder->Length();
          obs_bb = gz::math::AxisAlignedBox(
            obs_pose.Pos() - gz::math::Vector3d(radius, radius, height / 2),
            obs_pose.Pos() + gz::math::Vector3d(radius, radius, height / 2)
          );
        }
        else
        {
          ignmsg << "Entity " << obs_name->Data() << " has an unknown geometry" << std::endl;
          continue;
        }
        
        //double distance = aabb.Distance(act_aabb);
        //double distance = CalculateDistance(obs_bb, actor_bb);
        // Compute the closest point on the bounding box to the actor
        auto closest_point = ClosestPointOnBox(actor_pose.Pos(), obs_bb);
        double distance = (closest_point - actor_pose.Pos()).Length();
        // ignmsg << "Distance to the obstacle: " << distance << " m" << std::endl;
        if(distance < minDist)
        {
          //minDist = distance;
          //closest_obstacle = closest_point;
          //ignmsg << "Closest point on bounding box: " << closest_obstacle << std::endl;
          geometry_msgs::msg::Point point;
          point.x = closest_point.X();
          point.y = closest_point.Y();
          point.z = 0.0;
          pedestrians_[p.first].closest_obs.push_back(point);
        }
      }
      else 
      {
        // if the entity has no geometry component, we can just use the center position
        // and we build a default small bounding box around it.  
        //ignmsg << "Entity " << obs_name->Data() << " has no Geometry component" << std::endl;
        auto default_size = gz::math::Vector3d(0.35, 0.35, 1.0);
        gz::math::AxisAlignedBox obs_bb = gz::math::AxisAlignedBox(
          obs_pose.Pos() - default_size / 2,
          obs_pose.Pos() + default_size / 2
        );
        //double distance = aabb.Distance(act_aabb);
        //double distance = CalculateDistance(obs_bb, actor_bb);
        // Compute the closest point on the bounding box to the actor
        auto closest_point = ClosestPointOnBox(actor_pose.Pos(), obs_bb);
        double distance = (closest_point - actor_pose.Pos()).Length();
        //ignmsg << "Distance to the obstacle: " << distance << " m" << std::endl;
        if(distance < minDist)
        {
          //minDist = distance;
          //closest_obstacle = closest_point;
          //ignmsg << "Closest point on bounding box: " << closest_obstacle << std::endl;
          geometry_msgs::msg::Point point;
          point.x = closest_point.X();
          point.y = closest_point.Y();
          point.z = 0.0;
          pedestrians_[p.first].closest_obs.push_back(point);
        }
      }
    }
    firstObstaclePrint_ = false;

    // // we do not consider obstacles further than 10 m
    // if (minDist <= 10.0)
    // {
    //   geometry_msgs::msg::Point point;
    //   point.x = closest_obstacle.X();
    //   point.y = closest_obstacle.Y();
    //   point.z = 0.0;
    //   pedestrians_[p.first].closest_obs.push_back(point);
    // }
  }
}




/////////////////////////////////////////////////
/**
 * @brief Get the robot state from the simulation
 * @param _ecm The EntityComponentManager
 * @return true if the robot state was successfully retrieved
 */
bool HuNavSystemPluginIGN::getRobotState(const gz::sim::EntityComponentManager& _ecm, double _dt)
{
  // Robot model should have a WorldPose component
  gz::math::Pose3d pose = worldPose(robotEntity_, _ecm);
  pose.Pos().Z(0.0); // we set the Z coordinate to 0.0
  
  // 3D Pose
  gz::math::Pose3d prevPose(robotAgent_.position.position.x, robotAgent_.position.position.y, 0, 0, 0, robotAgent_.yaw);
  robotAgent_.position.position.x = pose.Pos().X();
  robotAgent_.position.position.y = pose.Pos().Y();
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, pose.Rot().Yaw());
  robotAgent_.position.orientation = tf2::toMsg(myQuaternion);
  robotAgent_.yaw = pose.Rot().Yaw();
  // Linear and angular velocities
  auto linvel = _ecm.Component<gz::sim::components::LinearVelocity>(robotEntity_);
  if(linvel)
  {
    gz::math::Vector3d lv = linvel->Data();
    robotAgent_.velocity.linear.x = lv.X();
    robotAgent_.velocity.linear.y = lv.Y();
    robotAgent_.linear_vel = lv.Length();
  }
  else
  {
    //RCLCPP_WARN(this->rosnode_->get_logger(), "Failed to get robot linear velocity. Computing manually...");
    gz::math::Vector3d linearVelocity;
    computeLinearVel(prevPose, pose, _dt, linearVelocity);
    robotAgent_.velocity.linear.x = linearVelocity.X();
    robotAgent_.velocity.linear.y = linearVelocity.Y();
    robotAgent_.linear_vel = linearVelocity.Length();
  }
  auto angvel = _ecm.Component<gz::sim::components::AngularVelocity>(robotEntity_);
  if(angvel)
  {
    gz::math::Vector3d av = angvel->Data();
    robotAgent_.velocity.angular.z = av.Z();
    robotAgent_.angular_vel = av.Z();
  }
  else
  {
    //RCLCPP_WARN(this->rosnode_->get_logger(), "Failed to get robot angular velocity. Computing manually...");
    gz::math::Vector3d angularVelocity;
    computeAngularVel(prevPose, pose, _dt, angularVelocity);
    robotAgent_.velocity.angular.z = angularVelocity.Z();
    robotAgent_.angular_vel = angularVelocity.Z();
  }
  //RCLCPP_INFO_STREAM(this->rosnode_->get_logger(), "Robot pos x: " << robotAgent_.position.position.x << ", y: " << robotAgent_.position.position.y << ", vel: " << robotAgent_.linear_vel);
  
  return true;
}



/////////////////////////////////////////////////
/**
 * @brief get the current status of all the pedestrians (not this actor) from the simulation
 * @param _ecm EntityComponentManager
 * @return true if the agent status was successfully retrieved
 */
bool HuNavSystemPluginIGN::getPedestrianStates(const gz::sim::EntityComponentManager& _ecm, double _dt)
{
  //for (unsigned int i = 0; i < pedestrians_.size(); ++i)
  for (auto& [pedEntity, pedAgent] : pedestrians_)
  {
    //gz::math::Pose3d pose = worldPose(pedEntity, _ecm);
    //gz::math::Pose3d pose = _ecm.Component<gz::sim::components::Pose>(pedEntity)->Data();
    auto traj = _ecm.Component<gz::sim::components::TrajectoryPose>(pedEntity);
    if(!traj)
    {
      ignerr << "ERROR! TrajectoryPose component does not exist for actor " << pedAgent.name << std::endl;
      continue;
    }
    gz::math::Pose3d pose = traj->Data();
    pose.Pos().Z(0.0); // we set the Z coordinate to 0.0
    //gz::math::Pose3d wp = worldPose(pedEntity, _ecm);
    //ignmsg << "GetPedestians: Actor [" << pedAgent.name << "] worldpose: " << wp << std::endl;
    //ignmsg << "GetPedestians: Actor [" << pedAgent.name << "] trajpose:  " << pose << std::endl;
    //double yaw = normalizeAngle(pose.Rot().Yaw() - M_PI_2);
    double yaw = normalizeAngle(pose.Rot().Yaw());
    ignition::math::Vector3d pos = pose.Pos();

    // Actors in Gazebo (specifically those that move using animations) do not have real physical dynamics 
    // like normal models. This means they do not automatically generate linear and angular velocity values 
    // in the EntityComponentManager (ECM), unlike models with enabled physical dynamics. Therefore, we 
    // cannot simply retrieve a LinearVelocity or AngularVelocity as we would with a normal model.
    // We do it manually.
    gz::math::Pose3d prevPose(pedAgent.position.position.x, pedAgent.position.position.y, 0, 0, 0, pedAgent.yaw);
    // Elapsed time
    //auto currentTime = std::chrono::steady_clock::now();
    //std::chrono::duration<double> elapsedTime = currentTime - pedLastTime_;
    //double dt = elapsedTime.count();
    //auto currentTime = this->rosnode_->get_clock()->now();
    //double dt = (currentTime - rosPedLastTime_).seconds();
    gz::math::Vector3d linearVelocity;
    computeLinearVel(prevPose, pose, _dt, linearVelocity);
    //RCLCPP_INFO_STREAM(this->rosnode_->get_logger(), "[" << pedAgent.name << "] lvel: " << linearVelocity.Length() << " m/s" << std::endl);
    //RCLCPP_INFO_STREAM(this->rosnode_->get_logger(), "Prev [" << prevPose.Pos().X() << ", " << prevPose.Pos().Y() << "], curr [" << pos.X() << ", " << pos.Y() << "]" << std::endl);
    gz::math::Vector3d angularVelocity;
    computeAngularVel(prevPose, pose, _dt, angularVelocity);
    //RCLCPP_INFO_STREAM(this->rosnode_->get_logger(), "Pedestrian [" << pedAgent.name << "] angular velocity: " << angularVelocity.Z() << "rad/s. dt: " << _dt << "secs" << std::endl);

    double xi = pedAgent.position.position.x;
    double yi = pedAgent.position.position.y;
    double xf = pos.X();
    double yf = pos.Y();
    double lvel = linearVelocity.Length();
    double vx = linearVelocity.X();
    double vy = linearVelocity.Y();
    double anvel = angularVelocity.Z();

    if (reset_)
    {
      lvel = 0.0;
      vx = 0.0;
      vy = 0.0;
      anvel = 0.0;
    }
    else if (lvel > pedAgent.desired_velocity)
    {
      lvel = pedAgent.desired_velocity;
      double maxd = lvel * update_rate_secs_;
      if (fabs(xf - xi) > maxd)
        vx = ((xf - xi) / fabs(xf - xi)) * maxd / _dt;
      if (fabs(yf - yi) > maxd)
        vy = ((yf - yi) / fabs(yf - yi)) * maxd / _dt;
    }
    pedAgent.velocity.linear.x = vx;
    pedAgent.velocity.linear.y = vy;
    pedAgent.linear_vel = lvel;
    pedAgent.velocity.angular.z = anvel;
    pedAgent.angular_vel = anvel;

    // Pose
    pedAgent.position.position.x = xf;  // pos.X();
    pedAgent.position.position.y = yf;  // pos.Y();
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, yaw);
    pedAgent.position.orientation = tf2::toMsg(myQuaternion);
    pedAgent.yaw = yaw;

    // RCLCPP_INFO(rosnode->get_logger(),
    //             "getPedestrians. Actor %s pose x: %.2f, y: %.2f, vx:%.2f, "
    //             "vy:%.2f, lv: %.2f, dvel: %.2f",
    //             pedAgent.name.c_str(),
    //             pedAgent.position.position.x,
    //             pedAgent.position.position.y,
    //             pedAgent.velocity.linear.x,
    //             pedAgent.velocity.linear.y, pedAgent.linear_vel,
    //             pedAgent.desired_velocity);

  }
  reset_ = false;
  return true;
  
}




void HuNavSystemPluginIGN::computeLinearVel(const gz::math::Pose3d& prevPose, const gz::math::Pose3d& currentPose, const double dt, gz::math::Vector3d& linearVelocity)
{
  // Get current time
  // auto currentTime = std::chrono::steady_clock::now();

  // Elapsed time
  //std::chrono::duration<double> elapsedTime = currentTime - prevTime;
  //double dt = elapsedTime.count();

  // Position difference
  gz::math::Vector3d deltaPosition = currentPose.Pos() - prevPose.Pos();
  // Linear velocity
  linearVelocity = deltaPosition / dt;
  //linearVelocity.Z(0.0); // Ignore Z component
  //ignmsg << "Delta position: " << deltaPosition << ", Linear velocity: " << linearVelocity << std::endl;

  // Update previous values
  //prevPose = currentPose;
  //prevTime = currentTime;
}


void HuNavSystemPluginIGN::computeAngularVel(const gz::math::Pose3d& prevPose, const gz::math::Pose3d& currentPose, const double dt, gz::math::Vector3d& angularVelocity)
{
  // Get current time
  // auto currentTime = std::chrono::steady_clock::now();

  // Elapsed time
  // std::chrono::duration<double> elapsedTime = currentTime - prevTime;
  // double dt = elapsedTime.count();

  // Orientation difference
  double deltaYaw = currentPose.Rot().Yaw() - prevPose.Rot().Yaw();

  // Angle normalization
  deltaYaw = atan2(sin(deltaYaw), cos(deltaYaw));

  // Angular velocity
  angularVelocity.Z(deltaYaw / dt);

  // Update previous values
  //prevPose = currentPose;
  //prevTime = currentTime;
}



void HuNavSystemPluginIGN::fixActorHeight(const hunav_msgs::msg::Agent& ag, gz::math::Pose3d& actorPose)
{
  actorPose.Pos().Z(0.35);
  // switch (ag.skin)
  // {
  //   // Elegant man
  //   case 0:
  //     actorPose.Pos().Z(0.35);
  //     break;
  //   // Casual man
  //   case 1:
  //     actorPose.Pos().Z(0.35);
  //     break;
  //   // Elegant woman
  //   case 2:
  //     actorPose.Pos().Z(0.35);
  //     break;
  //   // Regular man
  //   case 3:
  //     actorPose.Pos().Z(0.35);
  //     break;
  //   // Worker man
  //   case 4:
  //     actorPose.Pos().Z(0.35);
  //     break;
  //   // Balds
  //   case 5:
  //     actorPose.Pos().Z(0.35);
  //     break;
  //   case 6:
  //     actorPose.Pos().Z(0.35);
  //     break;
  //   case 7:
  //     actorPose.Pos().Z(0.35);
  //     break;
  //   case 8:
  //     actorPose.Pos().Z(0.35);
  //     break;
  //   default:
  //     break;
  // }
}



void HuNavSystemPluginIGN::updateGazeboPedestrians(gz::sim::EntityComponentManager& _ecm, const gz::sim::UpdateInfo& /*_info*/, const hunav_msgs::msg::Agents& _agents)
{
  // if (goalReceived == false)
  // {
  //   RCLCPP_INFO(rosnode->get_logger(), "HuNavPlugin. Waiting to receive the robot navigation goal...");
  //   return;
  // }

  // update the Gazebo actors
  for (auto a : _agents.agents)
  {
    auto entity = _ecm.EntityByComponents(gz::sim::components::Name(a.name));
    if(!entity)
    {
      RCLCPP_ERROR(rosnode_->get_logger(), "Pedestrian model '%s' not found!!!!", a.name.c_str());
      ignerr << "ERROR: Pedestrian model " << a.name << " not found! Shutting down ROS node" << std::endl;
      std::ostringstream oss;
      oss << "ERROR: Pedestrian model " << a.name << " not found!" << std::endl;
      throw std::runtime_error(oss.str());
      return;
    }

    gz::math::Pose3d actorPose; // = worldPose(entity, _ecm) is giving position zero!!!!
    auto apose = _ecm.Component<gz::sim::components::WorldPose>(entity);
    if(apose)
    {
      actorPose = apose->Data();
      // double di = sqrt(pow(actorPose.Pos().X() - a.position.position.x, 2) + pow(actorPose.Pos().Y() - a.position.position.y, 2));
      // RCLCPP_INFO(rosnode_->get_logger(), "Actor %s curr_pose: [%.4f, %.4f], new_pose: [%.4f, %.4f], D: %.4f", a.name.c_str(), actorPose.Pos().X(), actorPose.Pos().Y(), a.position.position.x, a.position.position.y, di);
    } else {
      RCLCPP_ERROR(rosnode_->get_logger(), "Actor %s does not have a WorldPose component", a.name.c_str());
      continue;
    }
    gz::math::Pose3d prevPose = actorPose;
    double yaw = a.yaw; //normalizeAngle(a.yaw);
    // I don't know why the yaw (visually) is not correct
    yaw -= 0.30;
    double currAngle = actorPose.Rot().Yaw();
    double diff = normalizeAngle(yaw - currAngle);
    if (std::fabs(diff) > IGN_DTOR(10)) //25 degrees to rads
    {
      yaw = normalizeAngle(currAngle + (diff * 0.01));  // 0.01, 0.005
    }
    //RCLCPP_INFO(rosnode_->get_logger(), "Actor %s yaw: %.2f", a.name.c_str(), a.yaw);

    // set the pose of the actor
    actorPose.Pos().X(a.position.position.x);
    actorPose.Pos().Y(a.position.position.y);
    actorPose.Pos().Z(0.8);
    //fixActorHeight(a, actorPose);
    // I have to add some pitch to show the agents properly
    actorPose.Rot() = ignition::math::Quaterniond(0, 0.35, yaw);


    // UPDATE TRAJECTORY POSE
    auto trajPoseComp = _ecm.Component<gz::sim::components::TrajectoryPose>(entity);
    // // Update actor root pose
    *trajPoseComp = gz::sim::components::TrajectoryPose(actorPose);
    // //_ecm.SetComponentData<gz::sim::components::TrajectoryPose>(entity, actorPose);
    // // Mark as a one-time-change so that the change is propagated to the GUI
    _ecm.SetChanged(entity, gz::sim::components::TrajectoryPose::typeId, gz::sim::ComponentState::OneTimeChange);

    // UPDATE WORLD POSE
    auto wpose = _ecm.Component<gz::sim::components::WorldPose>(entity);
    if(wpose)
    {
      wpose->Data() = actorPose;
      _ecm.SetChanged(entity, gz::sim::components::WorldPose::typeId, gz::sim::ComponentState::OneTimeChange);
    }
    else
    {
      ignwarn << "!!!!! UpdateAgents. WorldPose component does not exist for actor " << a.name << std::endl;
    }


    // update the pedestrians' goals and behavior
    pedestrians_[entity].goals.clear();
    pedestrians_[entity].goals = a.goals;
    if (a.behavior.state != pedestrians_[entity].behavior.state)
    {
      pedestrians_[entity].behavior.state = a.behavior.state;
    }


    // Distance traveled is used to coordinate motion with the walking
    // animation: newPose(actorPose) - prevPose(actor->WorldPose)
    double distanceTraveled = (actorPose.Pos() - prevPose.Pos()).Length();
    // if(counter_ == 500)
    //   RCLCPP_INFO_STREAM(rosnode_->get_logger(), "Agent " << a.name.c_str() << " Distance traveled: " << distanceTraveled << " m" << std::endl);

    // // TODO: select better animations for each behavior
    // // and adjust the animationFactor value for each case
    // double animationFactor;

    // if (a.behavior.state == hunav_msgs::msg::AgentBehavior::BEH_NO_ACTIVE)
    // {
    //   if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_REGULAR ||
    //       a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED ||
    //       a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
    //   {
    //     animationFactor = 1.0;
    //   }
    //   else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE)
    //   {
    //     animationFactor = 1.0;
    //   }
    //   else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
    //   {
    //     animationFactor = 1.0;
    //   }
    //   else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
    //   {
    //     animationFactor = 1.0;
    //   }
    //   else
    //   {
    //     animationFactor = 1.0;
    //   }
    // }
    // else
    // {
    //   if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_REGULAR)
    //   {
    //     animationFactor = 1.5;
    //   }
    //   else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE)
    //   {
    //     animationFactor = 1.5;
    //   }
    //   else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED)
    //   {
    //     animationFactor = 1.0;
    //   }
    //   else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
    //   {
    //     animationFactor = 1.0;
    //   }
    //   else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
    //   {
    //     animationFactor = 1.5;
    //   }
    //   else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
    //   {
    //     animationFactor = 1.0;
    //   }
    //   else
    //   {
    //     animationFactor = 1.0;
    //   }
    // }


    // Update actor bone trajectories based on animation time
    double animationFactor = 5.0; //0.005; //Noé
    auto animTimeComp = _ecm.Component<gz::sim::components::AnimationTime>(entity);
    //double animTimeInSeconds = std::chrono::duration_cast<std::chrono::duration<double>>(animTimeComp->Data()).count();
    //ignmsg << "Animation time: " << animTimeInSeconds << std::endl;
    auto animTime = animTimeComp->Data() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(distanceTraveled * animationFactor));
    //auto animTime =  std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    //  std::chrono::duration<double>(distanceTraveled * animationFactor));
    //ignmsg << "Animation time: " << animTime.count() << std::endl;
    *animTimeComp = gz::sim::components::AnimationTime(animTime);
    //_ecm.SetComponentData<gz::sim::components::AnimationTime>(entity, animTime);
    // Mark as a one-time-change so that the change is propagated to the GUI
    _ecm.SetChanged(entity, gz::sim::components::AnimationTime::typeId, gz::sim::ComponentState::OneTimeChange);


   
    // // change the animation
    // if (index > -1)
    // {
    //   gazebo::physics::TrajectoryInfoPtr trajectoryInfo;
    //   trajectoryInfo.reset(new gazebo::physics::TrajectoryInfo());
    //   trajectoryInfo->id = a.id;
    //   trajectoryInfo->duration = 1.0;
    //   if (a.behavior.state == hunav_msgs::msg::AgentBehavior::BEH_NO_ACTIVE)
    //   {
    //     trajectoryInfo->type = "no_active";
    //     // RCLCPP_INFO(rosnode->get_logger(),
    //     //            "changing behavior %i of %s to 'no_active'",
    //     //            (int)a.behavior, actor->GetName().c_str());
    //   }
    //   else
    //   {
    //     trajectoryInfo->type = "active";
    //     // RCLCPP_INFO(rosnode->get_logger(),
    //     //            "changing behavior %i of %s to 'active'",
    //     //            (int)a.behavior, actor->GetName().c_str());
    //   }
    //   actor->Stop();
    //   actor->SetCustomTrajectory(trajectoryInfo);
    //   actor->Play();
    // }
    // actor->SetScriptTime(actor->ScriptTime() + (distanceTraveled * animationFactor));
    // // lastUpdate = _info.simTime;
  }
}




/////////////////////////////////////////////////
/**
 * @brief post update method in which we call the HuNavSim service to compute the new agents status
 * @param _info UpdateInfo
 * @param _ecm EntityComponentManager
 */
void HuNavSystemPluginIGN::PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm)
{
  //(void)_info;
  //(void)_ecm;
  // std::string st;
  // auto pose = _ecm.Component<gz::sim::components::Pose>(actorEntity_)->Data();
  // //ignmsg << "Actor pose: " << pose << std::endl;
  // _info.paused ? st = "Simulation paused": st = "Simulation running";

  // _info.simTime;
  // //ignmsg << st << std::endl;
  // std_msgs::msg::String msg;
  // msg.data = st;
  // this->ros_test_pub_->publish(msg);

  //GZ_PROFILE("HuNavSystemPluginIGN::PreUpdate");

  counter_++;

  if (_info.paused) {
    if(counter_ == 1000)
    {
      ignmsg << "Simulation paused" << std::endl;
      counter_ = 0;
    }
    //lastUpdate_ = _info.simTime;
    return;
  }

  if(!agentsInitialized_)
  {
    initializeAgents(_ecm);
  }
  
  // Time delta
  std::chrono::duration<double> dtDuration = _info.simTime - lastUpdate_;
  double dt = dtDuration.count();

  lastUpdate_ = _info.simTime;

  // Convert _info.simTime to std::chrono::steady_clock::time_point
  //auto sTime = std::chrono::duration_cast<std::chrono::steady_clock::duration>(_info.simTime);
  //lastUpdate_ = std::chrono::steady_clock::time_point(sTime);
  rosLastUpdate_ = this->rosnode_->get_clock()->now();

  getObstacles(_ecm);

  if(!getRobotState(_ecm, dt))
  {
    RCLCPP_ERROR(this->rosnode_->get_logger(), "Failed to get robot state. Exiting...");
    ignerr << "Failed to get robot state. Exiting..." << std::endl;
    rclcpp::shutdown();
    return;
  }
  if(!getPedestrianStates(_ecm, dt))
  {
    RCLCPP_ERROR(this->rosnode_->get_logger(), "Failed to get pedestrian states. Exiting...");
    ignerr << "Failed to get pedestrian states. Exiting..." << std::endl;
    rclcpp::shutdown();
    return;
  }

  // Fill the service request
  auto request = std::make_shared<hunav_msgs::srv::ComputeAgents::Request>();

  // Wait for the service to be available
  while (!rosSrvClient_->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(rosnode_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_WARN(rosnode_->get_logger(), "Service /compute_agents not available, waiting again...");
  }

  hunav_msgs::msg::Agents agents;
  agents.header.frame_id = globalFrame_;
  // Extract seconds and nanoseconds from _info.simTime
  auto simTime = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
  auto simTimeNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime - simTime);
  builtin_interfaces::msg::Time ros_time;
  ros_time.sec = static_cast<int32_t>(simTime.count());
  ros_time.nanosec = static_cast<uint32_t>(simTimeNsec.count());
  agents.header.stamp = this->rosnode_->get_clock()->now(); //ros_time;
  std::vector<hunav_msgs::msg::Agent> vector;
  vector.reserve(pedestrians_.size()); // Reserve space to avoid multiple allocations
  for (const auto& pair : pedestrians_)
  {
    vector.push_back(pair.second);
  }
  agents.agents = vector;
  request->robot = robotAgent_;
  request->current_agents = agents;
  // for(auto agn : agents.agents)
  // {
  //   if(agn.name == std::string("agent1"))
  //   {
  //     RCLCPP_INFO(rosnode_->get_logger(), "Agent %s before. x: %.3f, y: %.3f, th: %.3f, lvel: %.3f",
  //             agn.name.c_str(), agn.position.position.x, agn.position.position.y,
  //             agn.yaw, agn.linear_vel);
  //     break;
  //   }
  // }
  

  // Call the service
  auto result = rosSrvClient_->async_send_request(request);
  // Wait for the result.
  std::chrono::duration<int, std::milli> ms(150);
  if (rclcpp::spin_until_future_complete(rosnode_, result, ms) ==
       rclcpp::FutureReturnCode::SUCCESS)
  {
    //if (rclcpp::spin_until_future_complete(this->rosnode_, result) ==
    //  rclcpp::FutureReturnCode::SUCCESS) {
    //auto res = result.wait_for(ms);
    //if (res == std::future_status::ready)
    
    hunav_msgs::msg::Agents updated_agents = result.get()->updated_agents;
      
    if(counter_ == 1000)
    {
      counter_ = 0;
      auto agents = updated_agents.agents;
      for (const auto& a : agents)
      {
        double goalx = a.goals[0].position.x;
        double goaly = a.goals[0].position.y;
        double currx = a.position.position.x;
        double curry = a.position.position.y;
        double dist = sqrt(pow(goalx - currx, 2) + pow(goaly - curry, 2));
        //RCLCPP_INFO(rosnode_->get_logger(), "Agent %s. Goal: %.2f, %.2f, Current pos:[%.2f, %.2f], Dist: %.2f\n",
        //  a.name.c_str(), goalx, goaly, currx, curry, dist);
      }
    }
    updateGazeboPedestrians(_ecm, _info, updated_agents);
  }
  // else if (res == std::future_status::timeout)
  // {
  //   RCLCPP_ERROR(rosnode_->get_logger(), "Service /compute_agents timeout!!");
  // }
  else
  {
    RCLCPP_ERROR(rosnode_->get_logger(), "Failed to call service /compute_agents");
  }
  
  
}




IGNITION_ADD_PLUGIN(HuNavSystemPluginIGN, gz::sim::System, HuNavSystemPluginIGN::ISystemConfigure, HuNavSystemPluginIGN::ISystemPreUpdate/*, HuNavPluginIGN::ISystemPostUpdate, HuNavPluginIGN::ISystemReset*/)

IGNITION_ADD_PLUGIN_ALIAS(HuNavSystemPluginIGN, "HuNavSystemPluginIGN")
