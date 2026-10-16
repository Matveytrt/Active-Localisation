#ifndef POSE_SAVER_PLUGIN_HPP
#define POSE_SAVER_PLUGIN_HPP

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Pose3.hh>
#include <fstream>
#include <string>

namespace gazebo
{
  /// \brief A plugin that saves robot base pose to TUM trajectory (.tum) files
  class PoseSaverPlugin : public ModelPlugin
  {
    /// \brief Constructor
    public: PoseSaverPlugin();

    /// \brief Destructor
    public: virtual ~PoseSaverPlugin();

    /// \brief Load the plugin
    /// \param[in] _parent Pointer to the parent model
    /// \param[in] _sdf Pointer to the SDF element
    public: virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Called every simulation iteration
    private: void OnUpdate();

    /// \brief Append current pose to the TUM trajectory (.tum) file
    private: void SavePose();

    /// \brief Generate filename with timestamp
    /// \return Filename with format robot_pose_YYYYMMDD_HHMMSS.tum
    private: std::string GenerateFilename();

    /// \brief Pointer to the model
    private: physics::ModelPtr model;

    /// \brief Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    /// \brief Save frequency in Hz
    private: double saveFrequency;

    /// \brief Directory path for output files
    private: std::string outputDir;

    /// \brief Time of last save operation
    private: common::Time lastSaveTime;

    /// \brief Output file stream
    private: std::ofstream outputFile;

    /// \brief Flag to track if header is written
    private: bool headerWritten;
  };
}

#endif // POSE_SAVER_PLUGIN_HPP
