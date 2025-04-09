/***********************************************************************/
/**                                                                    */
/** WorldGenerator.cpp                                                 */
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

#include "hunav_gazebo_fortress_wrapper/WorldGenerator.hpp"
//#include <ament_index_cpp/get_package_prefix.hpp>
//#include <ament_index_cpp/get_package_share_directory.hpp>

using namespace tinyxml2;

namespace hunav
{

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

WorldGenerator::WorldGenerator() : Node("hunav_gz_fortress_world_generator")
{
  // fill with the names of agent parameters
  params_ = { ".id",
              ".skin",
              ".behavior.type",
              ".behavior.configuration",
              ".behavior.duration",
              ".behavior.once",
              ".behavior.vel",
              ".behavior.dist",
              ".behavior.social_force_factor",
              ".behavior.goal_force_factor",
              ".behavior.obstacle_force_factor",
              ".behavior.other_force_factor",
              ".group_id",
              ".max_vel",
              ".radius",
              ".init_pose.x",
              ".init_pose.y",
              ".init_pose.z",
              ".init_pose.h",
              ".goal_radius",
              ".cyclic_goals",
              ".goals" };
  // names of the goal parameters
  goal_params_ = { ".x", ".y", ".h" };

    // 0 -> world plugin
  // 1 -> last actor plugin
  // 2 -> all actors plugin
  hunavplug_position_ = this->declare_parameter<int>("plugin_position", 0);

  agents_srv_ = this->create_service<hunav_msgs::srv::GetAgents>(
      std::string("get_agents"), std::bind(&hunav::WorldGenerator::getAgentsService, this, _1, _2));
  // agents_srv_ = this->create_service<hunav_msgs::srv::GetAgents>(
  //    std::string("get_agents"), &hunav::WorldGenerator::getAgentsService);

  // Read the plugin parameters
  if(!readPluginParams())
  {
    RCLCPP_ERROR(this->get_logger(), "Error reading plugin parameters!!!");
    return;
  }
  // Read the agents parameters
  if(!readAgentParams())
  {
    RCLCPP_ERROR(this->get_logger(), "Error reading agent parameters!!!");
    return;
  }
  // Generate the world
  if(!processXML())
  {
    RCLCPP_ERROR(this->get_logger(), "Error processing the XML file!!!");
    return;
  }
  RCLCPP_INFO(this->get_logger(), "WorldGenerator configured!!!");
}

WorldGenerator::~WorldGenerator()
{
}

bool WorldGenerator::readPluginParams()
{
  // Get the path to the worlds directory
  // std::string pkg_shared_dir;
  // try {
  //   pkg_shared_dir =
  //       ament_index_cpp::get_package_share_directory("hunav_gazebo_wrapper");

  // } catch (ament_index_cpp::PackageNotFoundError) {
  //   RCLCPP_ERROR(this->get_logger(),
  //                "Package hunav_gazebo_wrapper not found in dir: %s!!!",
  //                pkg_shared_dir.c_str());
  // }
  // pkg_shared_dir = pkg_shared_dir + "/worlds/";

  // get the world name
  // std::string world_file = this->declare_parameter<std::string>(
  //    "base_world", std::string("empty_cafe.world"));
  // base_world_ = pkg_shared_dir + world_file;

  // Plugin parameters
  base_world_ = this->declare_parameter<std::string>("base_world", std::string("bookstore.world"));
  plug_use_gazebo_obs_ = this->declare_parameter<bool>("use_gazebo_obs", false);
  plug_update_rate_ = this->declare_parameter<double>("update_rate", 1000.0);
  plug_robot_name_ = this->declare_parameter<std::string>("robot_name", std::string("robot"));
  RCLCPP_INFO(this->get_logger(), "Robot name: %s", plug_robot_name_.c_str());
  plug_global_frame_ = this->declare_parameter<std::string>("global_frame_to_publish", std::string("map"));
  plug_use_navgoal_to_start_ = this->declare_parameter<bool>("use_navgoal_to_start", false);
  plug_navgoal_topic_ = this->declare_parameter<std::string>("navgoal_topic", std::string("goal_pose"));
  this->declare_parameter(std::string("ignore_models"), rclcpp::ParameterType::PARAMETER_STRING);
  rclcpp::Parameter ig_models = this->get_parameter("ignore_models");
  std::string models = ig_models.as_string();
  plug_ignore_models_.push_back("link");
  plug_ignore_models_.push_back("visual");
  plug_ignore_models_.push_back("collision");
  plug_ignore_models_.push_back("surface");
  plug_ignore_models_.push_back("column");
  plug_ignore_models_.push_back("main_floor");
  plug_ignore_models_.push_back("sun");
  plug_ignore_models_.push_back("point_light");
  //plug_ignore_models_.push_back("column");
  // RCLCPP_INFO(this->get_logger(), "Ignore_models string: %s", models.c_str());
  const char delim = ' ';
  tokenize(models, delim, plug_ignore_models_);
  for (std::string st : plug_ignore_models_)
  {
    RCLCPP_INFO(this->get_logger(), "Ignore_model: %s", st.c_str());
  }
  return true;
}

bool WorldGenerator::readAgentParams()
{
  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this, "hunav_loader");
  while (!parameters_client->wait_for_service(1s))
  {
    if (!rclcpp::ok())
    {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  RCLCPP_INFO(this->get_logger(), "Reading parameters...");
  auto parameters = parameters_client->get_parameters({ "map", "agents" });

  std::string map = parameters[0].value_to_string();

  // std::cout << "map parameter: " << map << std::endl;
  // std::cout << "agent names: " << parameters[1].value_to_string() << std::endl << std::endl;

  //   for (auto &parameter : parameters) {
  //     std::cout << "\nParameter name: " << parameter.get_name() << std::endl;
  //     std::cout << "Parameter value (" << parameter.get_type_name()
  //               << "): " << parameter.value_to_string() << std::endl;
  //   }

  auto agent_names = parameters[1].as_string_array();
  for (std::string an : agent_names)
  {
    //std::cout << "agent name: " << an << std::endl;
    std::vector<std::string> agent_params = params_;
    for (unsigned int i = 0; i < params_.size(); i++)
    {
      agent_params[i] = an + agent_params[i];
      // std::cout << "agent_params " << i << ": " << agent_params[i] <<
      // std::endl;
    }
    auto aparams = parameters_client->get_parameters(agent_params);

    // {"agent1.id": {"type": "integer", "value": "1"},
    //"agent1.skin": {"type": "integer", "value": "0"},
    //"agent1.behavior.type": {"type": "integer", "value": "4"},
    //"agent1.behavior.configuration": {"type": "integer", "value": "0"},
    //"agent1.behavior.duration": {"type": "double", "value": "40.000000"},
    //"agent1.behavior.once": {"type": "bool", "value": "true"},
    //"agent1.behavior.vel": {"type": "double", "value": "0.600000"},
    //"agent1.behavior.dist": {"type": "double", "value": "3.000000"},
    //"agent1.behavior.social_force_factor": {"type": "double", "value": "5.000000"},
    //"agent1.behavior.goal_force_factor": {"type": "double", "value": "2.000000"},
    //"agent1.behavior.obstacle_force_factor": {"type": "double", "value": "10.000000"},
    //"agent1.behavior.other_force_factor": {"type": "double", "value": "20.000000"},
    //"agent1.group_id": {"type": "integer", "value": "-1"},
    //"agent1.max_vel": {"type": "double", "value": "1.500000"},
    //"agent1.radius": {"type": "double", "value": "0.400000"},
    //"agent1.init_pose.x": {"type": "double", "value": "3.484565"},
    //"agent1.init_pose.y": {"type": "double", "value": "6.934117"},
    //"agent1.init_pose.z": {"type": "double", "value": "1.250000"},
    //"agent1.init_pose.h": {"type": "double", "value": "0.000000"},
    //"agent1.goal_radius": {"type": "double", "value": "0.300000"},
    //"agent1.cyclic_goals": {"type": "bool", "value": "true"},
    //"agent1.goals": {"type": "string_array", "value": "[g0, g1, g2]"}}

    // std::cout << "aparams: " << aparams << std::endl;
    hunav_msgs::msg::Agent a;
    a.name = an;
    // std::cout << "aparams[0]: " << aparams[0] << std::endl;
    a.id = aparams[0].as_int();
    // std::cout << "id: " << a.id << std::endl;
    a.type = hunav_msgs::msg::Agent::PERSON;
    // std::cout << "aparams[1]: " << aparams[1] << std::endl;
    a.skin = aparams[1].as_int();
    // std::cout << "skin: " << a.skin << std::endl;

    // behavior
    a.behavior.type = aparams[2].as_int();
    a.behavior.configuration = aparams[3].as_int();
    a.behavior.duration = aparams[4].as_double();
    a.behavior.once = aparams[5].as_bool();
    a.behavior.vel = aparams[6].as_double();
    a.behavior.dist = aparams[7].as_double();
    a.behavior.social_force_factor = aparams[8].as_double();
    a.behavior.goal_force_factor = aparams[9].as_double();
    a.behavior.obstacle_force_factor = aparams[10].as_double();
    a.behavior.other_force_factor = aparams[11].as_double();

    // std::cout << "aparams[12]: " << aparams[12] << std::endl;
    a.group_id = aparams[12].as_int();
    // std::cout << "group_id: " << a.group_id << std::endl;
    // std::cout << "aparams[13]: " << aparams[13] << std::endl;
    a.desired_velocity = aparams[13].as_double();
    // std::cout << "desired vel: " << a.desired_velocity << std::endl;
    // std::cout << "aparams[14]: " << aparams[14] << std::endl;
    a.radius = aparams[14].as_double();
    // std::cout << "radius: " << a.radius << std::endl;

    // init pose
    a.position.position.x = aparams[15].as_double();
    a.position.position.y = aparams[16].as_double();
    a.position.position.z = aparams[17].as_double();
    a.yaw = aparams[18].as_double();
    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, aparams[9].as_double());
    a.position.orientation = tf2::toMsg(myQuaternion);
    a.goal_radius = aparams[19].as_double();
    a.cyclic_goals = aparams[20].as_bool();

    // std::cout << "id: " << a.id << " skin:" << (int)a.skin << " group_id:" << (int)a.group_id
    //           << " max_vel:" << a.desired_velocity << " radius:" << a.radius << std::endl
    //           << " initpose.x:" << a.position.position.x << " initpose.y:" << a.position.position.y << std::endl;
    // std::cout << "Behavior:" << std::endl;
    // std::cout << "type:" << (int)a.behavior.type << " configuration:" << (int)a.behavior.configuration
    //           << " duration:" << a.behavior.duration << " once:" << a.behavior.once << " vel:" << a.behavior.vel
    //           << " dist:" << a.behavior.dist << " goal_force_factor:" << a.behavior.goal_force_factor
    //           << " obstacle_force_factor:" << a.behavior.obstacle_force_factor
    //           << " social_force_factor:" << a.behavior.social_force_factor
    //           << " other_force_factor:" << a.behavior.other_force_factor << std::endl;

    auto goal_names = aparams[21].as_string_array();
    for (std::string goal : goal_names)
    {
      std::vector<std::string> gnames = goal_params_;
      for (unsigned int i = 0; i < goal_params_.size(); i++)
      {
        gnames[i] = an + "." + goal + goal_params_[i];
      }
      auto gparams = parameters_client->get_parameters({ gnames });
      geometry_msgs::msg::Pose p;
      p.position.x = gparams[0].as_double();
      p.position.y = gparams[1].as_double();
      tf2::Quaternion quat;
      quat.setRPY(0, 0, gparams[2].as_double());
      p.orientation = tf2::toMsg(quat);
      a.goals.push_back(p);
      //std::cout << "goal: " << goal << " x:" << p.position.x << " y:" << p.position.y << std::endl;
    }

    agents_.agents.push_back(a);
  }
  return true;
}

void WorldGenerator::getAgentsService(const std::shared_ptr<hunav_msgs::srv::GetAgents::Request> request,
                                      std::shared_ptr<hunav_msgs::srv::GetAgents::Response> response)
{
  (void)request;
  response->agents = agents_;
  std::cout << "Sending " << agents_.agents.size() << " agents to agent_manager" << std::endl;
  //std::cout << "Shutting down WorldGenerator..." << std::endl;
  // turn off the node since we do not use it anymore
  // rclcpp::shutdown();
}

bool WorldGenerator::processXML()
{
  // load the base world file
  tinyxml2::XMLDocument doc;
  const char* path = base_world_.c_str();
  std::cout << "Opening base Gazebo world: " << base_world_ << " ..." << std::endl;
  auto err = doc.LoadFile(path);

  if (err != tinyxml2::XML_SUCCESS)
  {
    RCLCPP_ERROR(this->get_logger(), "\nCould not open world file: %s", base_world_.c_str());
    RCLCPP_ERROR(this->get_logger(), "Please, check that the world file does not contain any comment!\n");
    return false;
  }

  // CREATE PHYSICS TAG
  if(!writePhysics(doc))
  {
    return false;
  }
  //std::cout << "Gazebo Physics written in file ..." << std::endl;

  // CREATE ROS2 PLUGINS TAGS
  if(!writeRos2Plugins(doc))
  {
    return false;
  }
  //std::cout << "ROS2 plugins written in file ..." << std::endl;

  // CREATE ACTORS
  if(!writeActors(doc))
  {
    return false;
  }
  std::cout << "Actors written in file ..." << std::endl;

  if(hunavplug_position_ == 0 || hunavplug_position_ == 1)
  {
    if(!writeHuNavPlugin(doc, hunavplug_position_))
    {
      return false;
    }
    std::cout << "HuNavPlugin written in file ..." << std::endl;
  }
  // FINALLY, SAVE THE NEW WORLD FILE
  std::size_t found = base_world_.find_last_of("/\\");
  // std::cout << " path: " << base_world_.substr(0, found + 1) << '\n';
  // std::cout << " file: " << base_world_.substr(found + 1) << '\n';
  std::string new_world = base_world_.substr(0, found) + "/" + "generatedWorld.sdf";
  std::cout << "Writing new sdf in file ..." << new_world << std::endl;
  if (doc.SaveFile(new_world.c_str()) == tinyxml2::XML_SUCCESS)
  {
    RCLCPP_INFO(this->get_logger(), "New Gazebo world file created! (%s)", new_world.c_str());
  }
  else
  {
    RCLCPP_ERROR(this->get_logger(), "Error while saving new Gazebo world file!");
    return false;
  }
  return true;
}


bool WorldGenerator::writePhysics(tinyxml2::XMLDocument &doc)
{
  // <physics type="ode">
  //   <max_step_size>0.01</max_step_size>
  //   <real_time_factor>1</real_time_factor>
  //   <real_time_update_rate>100</real_time_update_rate>
  // </physics>
  tinyxml2::XMLElement* physics_tag = doc.NewElement("physics");
  physics_tag->SetAttribute("type", "ode");
  tinyxml2::XMLElement* max_step = doc.NewElement("max_step_size");
  max_step->SetText(0.001);  // 0.01
  tinyxml2::XMLElement* time_factor = doc.NewElement("real_time_factor");
  time_factor->SetText(1);
  tinyxml2::XMLElement* time_rate = doc.NewElement("real_time_update_rate");
  time_rate->SetText(1000);  // 100

  // Check if the tag <physics> exists
  tinyxml2::XMLElement* physics =
      doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("physics");

  // if does not exist, we add it
  if (physics == nullptr)
  {
    // we add it
    // Insert physics in the XML
    doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertFirstChild(physics_tag);
    tinyxml2::XMLElement* phy = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("physics");
    phy->InsertFirstChild(max_step);
    phy->InsertAfterChild(max_step, time_factor);
    phy->InsertAfterChild(time_factor, time_rate);
  }
  else
  {
    tinyxml2::XMLElement* phy = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("physics");
    XMLElement* mss = phy->FirstChildElement("max_step_size");
    if (mss == nullptr)
    {
      phy->InsertFirstChild(max_step);
      mss = phy->FirstChildElement("max_step_size");
    }
    mss->SetText(0.001);  // 0.01

    XMLElement* rtf = phy->FirstChildElement("real_time_factor");
    if (rtf == nullptr)
    {
      phy->InsertAfterChild(max_step, time_factor);
      rtf = phy->FirstChildElement("real_time_factor");
    }
    rtf->SetText(1);

    XMLElement* rtur = phy->FirstChildElement("real_time_update_rate");
    if (rtur == nullptr)
    {
      phy->InsertAfterChild(time_factor, time_rate);
      rtur = phy->FirstChildElement("real_time_update_rate");
    }
    rtur->SetText(1000);  // 100
  }
  return true;
}


bool WorldGenerator::writeRos2Plugins(tinyxml2::XMLDocument &doc)
{
  // Physics plugin
  tinyxml2::XMLElement* physicsPlugin = doc.NewElement("plugin");
  physicsPlugin->SetAttribute("filename", "libignition-gazebo-physics-system.so");
  physicsPlugin->SetAttribute("name", "ignition::gazebo::systems::Physics");

  // Sensors plugin
  tinyxml2::XMLElement* sensorsPlugin = doc.NewElement("plugin");
  sensorsPlugin->SetAttribute("filename", "libignition-gazebo-sensors-system.so");
  sensorsPlugin->SetAttribute("name", "ignition::gazebo::systems::Sensors");
  tinyxml2::XMLElement* pRender = doc.NewElement("render_engine");
  pRender->SetText("ogre"); //ogre2

  // Commands plugin
  tinyxml2::XMLElement* cmdsPlugin = doc.NewElement("plugin");
  cmdsPlugin->SetAttribute("filename", "libignition-gazebo-user-commands-system.so");
  cmdsPlugin->SetAttribute("name", "ignition::gazebo::systems::UserCommands");

  // Broadcast plugin
  tinyxml2::XMLElement* brcastPlugin = doc.NewElement("plugin");
  brcastPlugin->SetAttribute("filename", "libignition-gazebo-scene-broadcaster-system.so");
  brcastPlugin->SetAttribute("name", "ignition::gazebo::systems::SceneBroadcaster");

  // Insert in the XML
  doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertFirstChild(brcastPlugin);
  doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertFirstChild(cmdsPlugin);
  doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertFirstChild(sensorsPlugin);
  tinyxml2::XMLElement* bcPlugin = doc.FirstChildElement("sdf")->FirstChildElement("world")->FirstChildElement("plugin");
  bcPlugin->InsertFirstChild(pRender);
  doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertFirstChild(physicsPlugin);

  return true;
}



bool WorldGenerator::writeActors(tinyxml2::XMLDocument &doc)
{

  std::string skin_filename[] = { "elegant_man.dae", "casual_man.dae", "elegant_woman.dae", "regular_man.dae",
                                  "worker_man.dae",  "walk.dae",       "walk-green.dae",    "walk-blue.dae",
                                  "walk-red.dae",    "stand.dae" };
  std::string animation_filename[] = { "07_01-walk.bvh",           "69_02_walk_forward.bvh",   "137_28-normal_wait.bvh",
                                       "142_01-walk_childist.bvh", "07_04-slow_walk.bvh",      "02_01-walk.bvh",
                                       "142_17-walk_scared.bvh",   "17_01-walk_with_anger.bvh" };


  // CREATE ACTORS
  //bool first = true;
  for (auto a : agents_.agents)
  {

    //  CREATING TAGS FOR THE ACTORS
    //name
    tinyxml2::XMLElement* pNewActor = doc.NewElement("actor");
    pNewActor->SetAttribute("name", a.name.c_str());
    // pose format = "x y z r p y"
    tinyxml2::XMLElement* pPose = doc.NewElement("pose");
    std::string pose = std::to_string(a.position.position.x) + " " + std::to_string(a.position.position.y) + " " +
                       std::to_string(a.position.position.z) + " 0 0 " + std::to_string(a.yaw);
    pPose->SetText(pose.c_str());
    // skin
    tinyxml2::XMLElement* pSkin = doc.NewElement("skin");
    // skin - filename
    tinyxml2::XMLElement* pFilename = doc.NewElement("filename");

    pFilename->SetText(std::string("models/"+skin_filename[a.skin]).c_str());
    // skin - scale
    tinyxml2::XMLElement* pScale = doc.NewElement("scale");
    pScale->SetText(1.0f);

    // animation name
    tinyxml2::XMLElement* pAnimation = doc.NewElement("animation");
    pAnimation->SetAttribute("name", "walk"); //"07_01-walk"); //"walk"); //"no_active"); //Noé
    // animation - filename
    tinyxml2::XMLElement* pFilename1 = doc.NewElement("filename");

    if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_REGULAR ||
        a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED ||
        a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
    {
      pFilename1->SetText(std::string("models/"+animation_filename[0]).c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE)
    {
      pFilename1->SetText(std::string("models/"+animation_filename[1]).c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
    {
      pFilename1->SetText(std::string("models/"+animation_filename[5]).c_str());
    }
    else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
    {
      pFilename1->SetText(std::string("models/"+animation_filename[1]).c_str());
    }

    //Noé - overwrite the model for the moment
    pFilename1->SetText(std::string("https://fuel.gazebosim.org/1.0/Mingfei/models/actor/tip/files/meshes/walk.dae").c_str());
    //pFilename1->SetText("/home/hunavsim_ws/src/hunav_gazebo_fortress_wrapper/worlds/models/07_01-walk.bvh");

    // animation - scale
    tinyxml2::XMLElement* pScale1 = doc.NewElement("scale");
    pScale1->SetText("1.0");
    // animation - interpolate
    tinyxml2::XMLElement* pInterpolate = doc.NewElement("interpolate_x");
    pInterpolate->SetText("true");

    // Set active animation
    // tinyxml2::XMLElement* pAnimation1 = doc.NewElement("animation");
    // pAnimation1->SetAttribute("name", "active");

    // tinyxml2::XMLElement* pFilename2 = doc.NewElement("filename");

    // if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_REGULAR)
    // {
    //   pFilename2->SetText(std::string("models/"+animation_filename[0]).c_str());
    // }
    // else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_IMPASSIVE)
    // {
    //   pFilename2->SetText(std::string("models/"+animation_filename[1]).c_str());
    // }
    // else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SURPRISED)
    // {
    //   pFilename2->SetText(std::string("models/"+animation_filename[2]).c_str());
    // }
    // else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_THREATENING)
    // {
    //   pFilename2->SetText(std::string("models/"+animation_filename[7]).c_str());
    // }
    // else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_SCARED)
    // {
    //   pFilename2->SetText(std::string("models/"+animation_filename[6]).c_str());
    // }
    // else if (a.behavior.type == hunav_msgs::msg::AgentBehavior::BEH_CURIOUS)
    // {
    //   pFilename2->SetText(std::string("models/"+animation_filename[4]).c_str());
    // }

    // tinyxml2::XMLElement* pScale2 = doc.NewElement("scale");
    // pScale2->SetText("1.0");
    // tinyxml2::XMLElement* pInterpolate1 = doc.NewElement("interpolate_x");
    // pInterpolate1->SetText("true");

    // WRITE TAGS IN THE XML FILE
    //std::cout << "Inserting first actor " << a.name <<  " in file ..." << std::endl;
     
      // tinyxml2::XMLElement* pInclude =
      //     doc.FirstChildElement("sdf")->LastChildElement("world"); //FirstChildElement("world")->LastChildElement("physics");
      
      // std::cout << "sdf->lastChild('world'): " << pInclude->GetText() <<std::endl;

      // if(pInclude == nullptr)
      // {
      //   std::cout << "Error getting last child of the world tag!!!" << std::endl;
      //   return false;
      // }
      // doc.FirstChildElement("sdf")->LastChildElement("world")->InsertAfterChild(pInclude, pNewActor);

    doc.FirstChildElement("sdf")->FirstChildElement("world")->InsertEndChild(pNewActor);
    //std::cout << "Inserted actor tag ..." << std::endl;

    tinyxml2::XMLElement* actortag =
        doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor");
    if(actortag == nullptr)
    {
      std::cout << "Error reading actor tag in the world file!!!" << std::endl;
      return false;
    }
    actortag->InsertFirstChild(pPose);
    actortag->InsertAfterChild(pPose, pSkin);
    //std::cout << "Inserted pose and skin ..." << std::endl;
    tinyxml2::XMLElement* skin =
        doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor")->FirstChildElement(
            "skin");
    if(skin == nullptr)
    {
      std::cout << "Error reading skin tag in the world file!!!" << std::endl;
      return false;
    }
    skin->InsertFirstChild(pFilename);
    skin->InsertAfterChild(pFilename, pScale);
    //std::cout << "Inserted scale ..." << std::endl;

    // Insert no_active animation
    actortag->InsertAfterChild(pSkin, pAnimation);
    tinyxml2::XMLElement* animation =
        doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor")->FirstChildElement(
            "animation");
    if(animation == nullptr)
    {
      std::cout << "Error reading animation tag in the world file!!!" << std::endl;
      return false;
    }
    animation->InsertFirstChild(pFilename1);
    animation->InsertAfterChild(pFilename1, pScale1);
    animation->InsertAfterChild(pScale1, pInterpolate); //--Noe
    //std::cout << "Inserted no-active animation ..." << std::endl;

    // // Insert active animation
    // actortag->InsertAfterChild(animation, pAnimation1);
    // tinyxml2::XMLElement* animation_active =
    //     doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor")->LastChildElement(
    //          "animation");
    // if(animation_active == nullptr)
    // {
    //   std::cout << "Error reading active animation tag in the world file!!!" << std::endl;
    //   return false;
    // }
    // animation_active->InsertFirstChild(pFilename2);
    // animation_active->InsertAfterChild(pFilename2, pScale2);
    // animation_active->InsertAfterChild(pScale2, pInterpolate1);
    // //std::cout << "Inserted active animation ..." << std::endl;

    // insert plugin in all the actors
    if(hunavplug_position_ == 2)
    {
      if(!writeHuNavPlugin(doc, hunavplug_position_))
      {
        return false;
      }
      std::cout << "HuNavPlugin written in file ..." << std::endl;
    }
  }
  return true;
}


bool WorldGenerator::writeHuNavPlugin(tinyxml2::XMLDocument &doc, int position)
{
  std::string filename = "libHuNavActorPluginIGN.so";
  std::string name = "HuNavActorPluginIGN";
  // 0 -> world plugin
  // 1 -> last actor plugin
  // 2 -> all actors plugin
  if(position == 0)
  {
    filename= "libHuNavSystemPluginIGN.so";
    name = "HuNavSystemPluginIGN";
  }

  // CREATE PLUGIN TAG 
  tinyxml2::XMLElement* pNewPlugin = doc.NewElement("plugin");
  pNewPlugin->SetAttribute("filename", filename.c_str());
  pNewPlugin->SetAttribute("name", name.c_str());

  tinyxml2::XMLElement* pUpdate = doc.NewElement("update_rate");
  pUpdate->SetText(plug_update_rate_);

  tinyxml2::XMLElement* pRobot = doc.NewElement("robot_name");
  pRobot->SetText(plug_robot_name_.c_str());

  tinyxml2::XMLElement* pGazebo = doc.NewElement("use_gazebo_obs");
  pGazebo->SetText(plug_use_gazebo_obs_);

  tinyxml2::XMLElement* pGlobalFrame = doc.NewElement("global_frame_to_publish");
  pGlobalFrame->SetText(plug_global_frame_.c_str());

  tinyxml2::XMLElement* pUseGoal = doc.NewElement("use_navgoal_to_start");
  pUseGoal->SetText(plug_use_navgoal_to_start_);
  tinyxml2::XMLElement* pGoalTopic = doc.NewElement("navgoal_topic");
  pGoalTopic->SetText(plug_navgoal_topic_.c_str());

  tinyxml2::XMLElement* pIgnoreModels = doc.NewElement("ignore_models");
  std::vector<tinyxml2::XMLElement*> pModels;
  for (std::string st : plug_ignore_models_)
  {
    // tinyxml2::XMLElement *pm = doc.NewElement("model");
    pModels.push_back(doc.NewElement("model"));
    pModels.back()->SetText(st.c_str());
  }

  // Insert plugin at the end of the XML
  auto pos = doc.FirstChildElement("sdf")->FirstChildElement("world");
  if(position == 0)
    pos->InsertEndChild(pNewPlugin);
  if(position == 1 || position == 2)
  {
    // Insert plugin in the last actor of the XML
    pos = doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("actor");
    pos->InsertFirstChild(pNewPlugin);
  } 

  tinyxml2::XMLElement* plugin = pos->LastChildElement("plugin");
  //tinyxml2::XMLElement* plugin = doc.FirstChildElement("sdf")->FirstChildElement("world")->LastChildElement("plugin");
  plugin->InsertFirstChild(pUpdate);
  plugin->InsertAfterChild(pUpdate, pRobot);
  plugin->InsertAfterChild(pRobot, pGazebo);
  plugin->InsertAfterChild(pGazebo, pGlobalFrame);
  plugin->InsertAfterChild(pGlobalFrame, pUseGoal);
  plugin->InsertAfterChild(pUseGoal, pGoalTopic);
  plugin->InsertAfterChild(pGoalTopic, pIgnoreModels);
  tinyxml2::XMLElement* ignore =
      pos->LastChildElement("plugin")->FirstChildElement("ignore_models");

  if (!pModels.empty())
  {
    ignore->InsertFirstChild(pModels[0]);
    for (size_t i = 1; i < pModels.size(); i++)
      ignore->InsertAfterChild(pModels[i - 1], pModels[i]);
  }
  return true;
}




}  // namespace hunav

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<hunav::WorldGenerator>());

  rclcpp::shutdown();
  return 0;
}
