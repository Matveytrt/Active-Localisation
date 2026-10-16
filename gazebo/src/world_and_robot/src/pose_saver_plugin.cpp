#include "pose_saver_plugin.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <sys/stat.h>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(PoseSaverPlugin)

  /////////////////////////////////////////////////
  PoseSaverPlugin::PoseSaverPlugin() 
    : saveFrequency(10.0), headerWritten(false)
  {
  }

  /////////////////////////////////////////////////
  PoseSaverPlugin::~PoseSaverPlugin()
  {
    // Close the output file if it's open
    if (this->outputFile.is_open())
    {
      this->outputFile.close();
      gzmsg << "PoseSaverPlugin: Closed output file" << std::endl;
    }
  }

  /////////////////////////////////////////////////
  void PoseSaverPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
  {
    // Store the pointer to the model
    this->model = _parent;

    // Get parameters from SDF
    if (_sdf->HasElement("save_frequency"))
    {
      this->saveFrequency = _sdf->Get<double>("save_frequency");
    }

    if (_sdf->HasElement("output_dir"))
    {
      this->outputDir = _sdf->Get<std::string>("output_dir");
    }
    else
    {
      // Default directory when output_dir is not set in SDF
      this->outputDir = "/tmp/robot_poses";
    }

    // Create directory if it doesn't exist
    struct stat st;
    if (stat(this->outputDir.c_str(), &st) != 0)
    {
      mkdir(this->outputDir.c_str(), 0755);
      gzmsg << "PoseSaverPlugin: Created directory " << this->outputDir << std::endl;
    }

    // Generate filename and open file
    std::string filename = this->GenerateFilename();
    this->outputFile.open(filename, std::ios::out);

    if (!this->outputFile.is_open())
    {
      gzerr << "PoseSaverPlugin: Failed to open file " << filename << std::endl;
      return;
    }

    gzmsg << "PoseSaverPlugin: Saving poses to " << filename << std::endl;
    gzmsg << "PoseSaverPlugin: Save frequency: " << this->saveFrequency << " Hz" << std::endl;

    // Write TUM format header (as comment)
    this->outputFile << "# TUM trajectory format" << std::endl;
    this->outputFile << "# timestamp x y z qx qy qz qw" << std::endl;
    this->headerWritten = true;

    // Initialize last save time
    this->lastSaveTime = this->model->GetWorld()->SimTime();

    // Listen to the update event
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&PoseSaverPlugin::OnUpdate, this));

    gzmsg << "PoseSaverPlugin: Initialized successfully for model '"
          << this->model->GetName() << "'" << std::endl;
  }

  /////////////////////////////////////////////////
  void PoseSaverPlugin::OnUpdate()
  {
    // Get current simulation time
    common::Time currentTime = this->model->GetWorld()->SimTime();
    
    // Calculate time difference
    double dt = (currentTime - this->lastSaveTime).Double();
    
    // Check if it's time to save (based on frequency)
    double saveInterval = 1.0 / this->saveFrequency;
    
    if (dt >= saveInterval)
    {
      this->SavePose();
      this->lastSaveTime = currentTime;
    }
  }

  /////////////////////////////////////////////////
  void PoseSaverPlugin::SavePose()
  {
    if (!this->outputFile.is_open())
    {
      return;
    }

    // Get current simulation time
    common::Time currentTime = this->model->GetWorld()->SimTime();
    
    // Get world pose of the model
    ignition::math::Pose3d pose = this->model->WorldPose();
    
    // Extract position
    ignition::math::Vector3d position = pose.Pos();
    double x = position.X();
    double y = position.Y();
    double z = position.Z();
    
    // Extract orientation as quaternion
    ignition::math::Quaterniond orientation = pose.Rot();
    double qx = orientation.X();
    double qy = orientation.Y();
    double qz = orientation.Z();
    double qw = orientation.W();
    
    // Write to TUM format file with high precision
    // Format: timestamp x y z qx qy qz qw
    this->outputFile << std::fixed << std::setprecision(6)
                     << currentTime.Double() << " "
                     << x << " "
                     << y << " "
                     << z << " "
                     << qx << " "
                     << qy << " "
                     << qz << " "
                     << qw << std::endl;
    
    // Flush to ensure data is written
    this->outputFile.flush();
  }

  /////////////////////////////////////////////////
  std::string PoseSaverPlugin::GenerateFilename()
  {
    // Get current real-world time
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);
    
    // Format: robot_pose_YYYYMMDD_HHMMSS.tum
    std::ostringstream oss;
    oss << this->outputDir << "/robot_pose_"
        << std::setfill('0')
        << std::setw(4) << (localTime->tm_year + 1900)
        << std::setw(2) << (localTime->tm_mon + 1)
        << std::setw(2) << localTime->tm_mday
        << "_"
        << std::setw(2) << localTime->tm_hour
        << std::setw(2) << localTime->tm_min
        << std::setw(2) << localTime->tm_sec
        << ".tum";
    
    return oss.str();
  }
}
