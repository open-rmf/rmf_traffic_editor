/*
 * Copyright (C) 2020 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <string>
#include <functional>
#include <ignition/transport/Node.hh>
#include <opencv2/opencv.hpp>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/transport/transport.hh"
#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/rendering.hh"
#include "gazebo/util/system.hh"

using namespace gazebo;

class GZ_PLUGIN_VISIBLE ThumbnailGenerator : public SystemPlugin
{

public:
  /////////////////////////////////////////////
  virtual ~ThumbnailGenerator()
  {
    rendering::fini();
  }

  /////////////////////////////////////////////
  void Load(int _argc, char** _argv)
  {
    if (_argc != 6)
    {
      std::cerr << " Error: Invalid Number of Arguments\n";
      this->PrintHelp();
      _exit_flag = true;
      return;
    }

    // load directory path from arg
    std::string model_path = _argv[_argc - 2];
    std::string output_dir = _argv[_argc - 1];
    std::cout << " - Input Model Path: " << model_path << std::endl;
    std::cout << " - Output Directory: " << output_dir << std::endl;

    // folder and files management
    _output_path = boost::filesystem::path(output_dir);
    if (!boost::filesystem::exists(_output_path))
      boost::filesystem::create_directories(_output_path);

    std::ifstream ifs(model_path.c_str());
    if (!ifs)
    {
      std::cerr << "Error: Unable to open file[" << model_path << "]\n";
      this->PrintHelp();
      _exit_flag = true;
      return;
    }

    this->_sdf_model.reset(new sdf::SDF());
    if (!sdf::init(this->_sdf_model))
    {
      std::cerr << "ERROR: SDF parsing the xml failed" << std::endl;
      this->PrintHelp();
      _exit_flag = true;
      return;
    }

    if (!sdf::readFile(model_path, this->_sdf_model))
    {
      std::cerr << "Error: SDF parsing the xml failed\n";
      this->PrintHelp();
      _exit_flag = true;
      return;
    }

    sdf::ElementPtr model_elem = this->_sdf_model->Root()->GetElement("model");
    this->_model_name = model_elem->Get<std::string>("name");
    _exit_flag = false;
  }

  /////////////////////////////////////////////
  void Init()
  {
    gazebo::sensors::disable();

    this->_world_created_conn = event::Events::ConnectWorldCreated(
      std::bind(&ThumbnailGenerator::OnWorldCreated, this));

    this->_update_conn = event::Events::ConnectWorldUpdateBegin(
      std::bind(&ThumbnailGenerator::Update, this));

    this->_transport_node = transport::NodePtr(new transport::Node());
    this->_transport_node->Init();
    this->_factory_pub = this->_transport_node->Advertise<msgs::Factory>(
      "~/factory");
    this->_server_control_pub =
      this->_transport_node->Advertise<msgs::ServerControl>(
      "/gazebo/server/control");
  }

  /////////////////////////////////////////////
  void OnWorldCreated()
  {
    this->_factory_pub->WaitForConnection();

    if (this->_sdf_model)
    {
      msgs::Factory msg;
      msg.set_sdf(this->_sdf_model->ToString());
      this->_factory_pub->Publish(msg, true);
    }
  }

  /////////////////////////////////////////////
  void Update()
  {
    if (_exit_flag)
    {
      // Clean up Connections and Cameras
      this->_world_created_conn.reset();
      this->_update_conn.reset();
      this->_camera.reset();

      // Tell the server to stop.
      msgs::ServerControl msg;
      msg.set_stop(true);
      this->_server_control_pub->Publish(msg);
      printf(" Done, Exiting \n");
      return;
    }

    // Make sure to initialize the rendering engine in the same thread that will
    // capture images.
    if (!this->_scene)
    {
      printf(" Initializing Thumbnails Gen Plugin \n");
      rendering::load();
      rendering::init();

      sdf::ElementPtr cameraSDF(new sdf::Element);
      sdf::initFile("camera.sdf", cameraSDF);

      this->_scene = rendering::create_scene("default", false, true);
      this->_camera = this->_scene->CreateCamera("viewing_cam", true);
      this->_camera->SetCaptureData(true);
      this->_camera->Load(cameraSDF);
      this->_camera->Init();
      this->_camera->SetHFOV(static_cast<ignition::math::Angle>(0.04));
      this->_camera->SetImageWidth(2000);
      this->_camera->SetImageHeight(2000);
      this->_camera->CreateRenderTexture("RenderTex");
      this->_camera->SetClipDist(100, 1000);

      gazebo::rendering::RTShaderSystem::Instance()->UpdateShaders();
      return;
    }

    // Main Thumbnail generation
    if (this->_camera && this->_scene)
    {
      printf(" Generating Thumbnail for %s \n", this->_model_name.c_str());
      event::Events::preRender();

      // todo: green fringe?? can add white background easily here
      // green background
      this->_scene->SetAmbientColor(ignition::math::Color(1, 1, 1, 1));
      this->_scene->SetBackgroundColor(ignition::math::Color(0, 1, 0, 1));
      this->_scene->SetShadowsEnabled(false);
      this->RenderCameraVisual();

      // Get Image data from camera in scene
      const unsigned char* img_data_ptr = this->_camera->ImageData();
      unsigned char* img_ptr;
      img_ptr = (unsigned char*)img_data_ptr;
      cv::Mat green_img(cv::Size(2000, 2000), CV_8UC3, img_ptr);
      cv::cvtColor(green_img, green_img, cv::COLOR_BGR2RGB);

      // Create Mask acording to green background
      cv::Mat green_mask;
      cv::inRange(green_img, cv::Scalar(0, 230, 0),
        cv::Scalar(10, 255, 10), green_mask);
      cv::bitwise_not(green_mask, green_mask);

      // Created masked img with alpha val, then crop it!
      cv::Mat masked_img(cv::Size(2000, 2000), CV_8UC4);
      cv::cvtColor(green_img, masked_img, cv::COLOR_RGB2RGBA);
      std::vector<cv::Mat> channels;
      cv::split(green_img, channels);
      channels.push_back(green_mask);
      cv::merge(channels, masked_img);
      cv::Rect r = cv::boundingRect(channels[3]);

      // output thumbnail png
      std::string img_name = this->_model_name + ".png";
      cv::imwrite((_output_path / img_name).string(), masked_img(r));
      std::cout << img_name << " saved!" << std::endl;
      _exit_flag = true;
    }
  }

  /////////////////////////////////////////////
  void RenderCameraVisual()
  {
    rendering::VisualPtr vis = this->_scene->GetVisual(this->_model_name);
    // Place the visual at the origin
    ignition::math::Box bbox = vis->BoundingBox();
    ignition::math::Vector3d trans = bbox.Center();
    vis->SetWorldPose(
      ignition::math::Pose3d(trans.X(), trans.Y(), trans.Z(), 0, 0, 0));
    bbox = vis->BoundingBox();

    // Generate Top view PNG Img
    ignition::math::Pose3d pose;
    pose.Pos().Set(0, 0, 200);
    pose.Rot().Euler(0, IGN_DTOR(90), 0);
    this->_camera->SetWorldPose(pose);
    this->_camera->Update();
    this->_camera->Render(true);
    this->_camera->PostRender();
  }

  /////////////////////////////////////////////
  void PrintHelp()
  {
    std::cout << "Usage: gzserver -s libthumbnail_gen.so empty.world "
              << "${INPUT_SDF_FILE} ${OUTPUT_FOLDER} \n"
              << std::endl;
    std::cout << "  arg: INPUT_SDF_FILE: "
              << "path to a model's sdf file" << std::endl;
    std::cout << "  arg: OUTPUT_FOLDER:  "
              << "path to ouput the cropped thumbnail \n"
              << std::endl;
  }

private:
  event::ConnectionPtr _update_conn;
  event::ConnectionPtr _world_created_conn;
  transport::NodePtr _transport_node;
  transport::PublisherPtr _server_control_pub;
  transport::PublisherPtr _factory_pub;
  rendering::ScenePtr _scene;
  rendering::CameraPtr _camera;
  sdf::SDFPtr _sdf_model;
  bool _exit_flag;

  std::string _model_name;
  boost::filesystem::path _output_path;
};

// Register this plugin with the simulator
GZ_REGISTER_SYSTEM_PLUGIN(ThumbnailGenerator)
