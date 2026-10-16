#!/usr/bin/env python3
"""
Experiment Orchestrator Node

Main orchestration node that manages experiment execution lifecycle.
Uses subprocess to launch and stop the STag detector with logging.
"""

import itertools
import os
import signal
import subprocess
import time
import yaml
from pathlib import Path
from typing import List, Dict, Any, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .experiment_state_manager import ExperimentStateManager, ExperimentState, ExperimentInfo
from .path_generator import PathGenerator


class OrchestratorNode(Node):
    """
    Orchestrates automated STag marker detection experiments.
    
    Workflow:
    1. Load configuration and generate experiment queue
    2. Wait for START command from keyboard control
    3. Launch detector+logger with appropriate parameters
    4. Wait for STOP command
    5. Stop detector+logger
    6. Move to next experiment
    """
    
    def __init__(self):
        super().__init__('experiment_orchestrator')
        
        # Declare and load parameters
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').value
        
        if not config_file:
            self.get_logger().error('config_file parameter is required!')
            raise ValueError('config_file parameter not provided')
        
        self.config = self._load_config(config_file)
        
        # Initialize components
        self.state_manager = ExperimentStateManager()
        self.path_generator = PathGenerator(
            base_path=self.config['output']['base_path'],
            bags_subdir=self.config['output']['bags_subdir'],
            logs_subdir=self.config['output']['logs_subdir']
        )
        
        self.experiment_queue = self._generate_experiment_queue()
        self.current_experiment_index = 0
        
        # Process management
        self.detector_process: Optional[subprocess.Popen] = None
        
        # ROS communication
        self.control_sub = self.create_subscription(
            String,
            '/experiment/control',
            self.control_callback,
            10
        )
        
        # Status publishing timer
        self.status_timer = self.create_timer(2.0, self.publish_status)
        
        # Initialize state
        self.state_manager.transition_to(ExperimentState.READY)
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('EXPERIMENT ORCHESTRATOR INITIALIZED')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Total experiments: {len(self.experiment_queue)}')
        self.get_logger().info(f'Output directory: {self.config["output"]["base_path"]}')
        self.get_logger().info('')
        self.get_logger().info('Press SPACE in the keyboard control terminal to start first experiment')
        self.get_logger().info('=' * 70)
    
    def _load_config(self, config_file: str) -> Dict[str, Any]:
        """Load configuration from YAML file."""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
            self.get_logger().info(f'Configuration loaded from: {config_file}')
            return config
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            raise
    
    def _generate_experiment_queue(self) -> List[Tuple[str, str, str, int]]:
        """
        Generate queue of all experiments to run.
        
        Returns:
            List of tuples: (marker, experiment_type, gait_type, attempt)
        """
        experiments = self.config['experiments']
        
        markers = experiments['markers']
        experiment_types = experiments['experiment_types']
        gait_types = experiments['gait_types']
        attempts = range(1, experiments['attempts_per_combination'] + 1)
        
        # Generate all combinations
        queue = list(itertools.product(markers, experiment_types, gait_types, attempts))
        
        return queue
    
    def get_current_experiment(self) -> Optional[Tuple[str, str, str, int]]:
        """Get current experiment from queue."""
        if self.current_experiment_index < len(self.experiment_queue):
            return self.experiment_queue[self.current_experiment_index]
        return None
    
    def control_callback(self, msg: String):
        """Handle control commands from keyboard."""
        command = msg.data
        
        self.get_logger().info(f'Received command: {command}')
        
        if command == 'start_stop':
            self._handle_start_stop()
        elif command == 'next':
            self._handle_next()
        elif command == 'retry':
            self._handle_retry()
        elif command == 'pause':
            self._handle_pause()
        elif command == 'quit':
            self._handle_quit()
        elif command == 'skip':
            self._handle_skip()
    
    def _handle_start_stop(self):
        """Handle START/STOP command."""
        if self.state_manager.state == ExperimentState.READY:
            # Start experiment
            self._start_current_experiment()
        elif self.state_manager.state == ExperimentState.RUNNING:
            # Stop experiment
            self._stop_current_experiment()
    
    def _handle_next(self):
        """Handle NEXT command (skip to next experiment)."""
        if self.state_manager.state == ExperimentState.READY:
            self.current_experiment_index += 1
            if self.current_experiment_index >= len(self.experiment_queue):
                self.get_logger().info('All experiments completed!')
                self.state_manager.transition_to(ExperimentState.COMPLETED)
            else:
                self.get_logger().info(f'Skipped to experiment {self.current_experiment_index + 1}')
    
    def _handle_retry(self):
        """Handle RETRY command (repeat current experiment)."""
        if self.state_manager.state == ExperimentState.READY:
            self.get_logger().info('Retrying current experiment')
            # Just stay at current index, will restart on next START command
    
    def _handle_pause(self):
        """Handle PAUSE command."""
        if self.state_manager.state == ExperimentState.READY:
            self.state_manager.pause_series()
            self.get_logger().info('Experiment series PAUSED. Press P again to resume.')
        elif self.state_manager.state == ExperimentState.PAUSED:
            self.state_manager.resume_series()
            self.get_logger().info('Experiment series RESUMED')
    
    def _handle_quit(self):
        """Handle QUIT command."""
        self.get_logger().info('QUIT command received. Stopping all experiments...')
        if self.state_manager.state == ExperimentState.RUNNING:
            self._stop_detector_process()
        self.state_manager.transition_to(ExperimentState.COMPLETED)
        rclpy.shutdown()
    
    def _handle_skip(self):
        """Handle SKIP command (after error)."""
        if self.state_manager.state == ExperimentState.ERROR:
            self.get_logger().info('Skipping failed experiment...')
            self.current_experiment_index += 1
            if self.current_experiment_index >= len(self.experiment_queue):
                self.state_manager.transition_to(ExperimentState.COMPLETED)
            else:
                self.state_manager.transition_to(ExperimentState.READY)
    
    def _start_current_experiment(self):
        """Start the current experiment from the queue."""
        experiment = self.get_current_experiment()
        
        if not experiment:
            self.get_logger().info('No more experiments in queue!')
            self.state_manager.transition_to(ExperimentState.COMPLETED)
            return
        
        marker, exp_type, gait, attempt = experiment
        
        # Create ExperimentInfo
        exp_info = ExperimentInfo(
            marker=marker,
            experiment_type=exp_type,
            gait_type=gait,
            attempt=attempt,
            total_experiments=len(self.experiment_queue),
            completed_count=self.current_experiment_index
        )
        
        # Update state
        if not self.state_manager.start_experiment(exp_info):
            self.get_logger().error('Failed to start experiment (invalid state)')
            return
        
        # Generate paths
        paths = self.path_generator.get_absolute_paths(
            marker, exp_type, gait, attempt,
            create_dirs=self.config['orchestrator'].get('auto_create_dirs', True)
        )
        
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'STARTING EXPERIMENT {self.current_experiment_index + 1}/{len(self.experiment_queue)}')
        self.get_logger().info('=' * 70)
        self.get_logger().info(f'Marker:      {marker}')
        self.get_logger().info(f'Library:     {paths["library_name"]}')
        self.get_logger().info(f'Experiment:  {exp_type}')
        self.get_logger().info(f'Gait:        {gait}')
        self.get_logger().info(f'Attempt:     {attempt}')
        self.get_logger().info(f'Bags dir:    {paths["bags_dir"]}')
        self.get_logger().info(f'Logs dir:    {paths["logs_dir"]}')
        self.get_logger().info('=' * 70)
        
        # Launch detector+logger
        success = self._launch_detector(paths)
        
        if not success:
            self.state_manager.set_error('Failed to launch detector')
            self.get_logger().error('Failed to launch detector! Press R to retry or S to skip')
        else:
            self.get_logger().info('Experiment RUNNING. Press SPACE to stop.')
    
    def _stop_current_experiment(self):
        """Stop the currently running experiment."""
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('STOPPING EXPERIMENT')
        self.get_logger().info('=' * 70)
        
        # Stop detector process
        self._stop_detector_process()
        
        # Cooldown
        cooldown = self.config['orchestrator'].get('cooldown_time', 2.0)
        self.get_logger().info(f'Cooldown for {cooldown} seconds...')
        time.sleep(cooldown)
        
        # Move to next experiment
        self.current_experiment_index += 1
        
        # Update state
        self.state_manager.mark_completed()
        
        if self.current_experiment_index >= len(self.experiment_queue):
            self.get_logger().info('')
            self.get_logger().info('=' * 70)
            self.get_logger().info('ALL EXPERIMENTS COMPLETED!')
            self.get_logger().info('=' * 70)
        else:
            next_exp = self.get_current_experiment()
            if next_exp:
                marker, exp_type, gait, attempt = next_exp
                self.get_logger().info(f'Ready for next: {marker}_{exp_type}_{gait}_{attempt}')
                self.get_logger().info('Press SPACE to start')
    
    def _launch_detector(self, paths: Dict[str, str]) -> bool:
        """
        Launch detector+logger subprocess.
        
        Args:
            paths: Dictionary with all paths and filenames
        
        Returns:
            True if launched successfully
        """
        try:
            launch_file = self.config['stag_detector']['launch_file']
            
            # Build ros2 launch command
            cmd = [
                'ros2', 'launch',
                launch_file,
                f'detector_config:={self.config["stag_detector"]["detector_config"]}',
                f'marker_type:={paths["library_name"]}',  # Pass library name as marker type
                f'log_file_path:={paths["log_file"]}',
                f'rosbag_output_dir:={paths["bags_dir"]}',
                f'rosbag_name:={paths["bag_filename"].replace(".db3", "")}',
                f'rosbag_topics:={self.config["stag_detector"]["rosbag_topics"]}',
                f'enable_logging:=true',
                f'enable_rosbag:=true',
            ]
            
            self.get_logger().info(f'Launching with marker_type: {paths["library_name"]}')
            self.get_logger().info(f'Command: {" ".join(cmd)}')
            
            # Start process
            self.detector_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                preexec_fn=os.setsid  # Create new process group for clean termination
            )
            
            # Give it a moment to start
            time.sleep(1.0)
            
            # Check if process is still running
            if self.detector_process.poll() is not None:
                self.get_logger().error('Detector process terminated immediately')
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Failed to launch detector: {e}')
            return False
    
    def _stop_detector_process(self):
        """Stop the detector subprocess gracefully."""
        if self.detector_process and self.detector_process.poll() is None:
            try:
                # Send SIGINT to process group for graceful shutdown
                os.killpg(os.getpgid(self.detector_process.pid), signal.SIGINT)
                
                # Wait for termination
                self.detector_process.wait(timeout=5.0)
                self.get_logger().info('Detector process stopped gracefully')
                
            except subprocess.TimeoutExpired:
                # Force kill if it doesn't stop
                self.get_logger().warn('Detector process did not stop gracefully, forcing...')
                os.killpg(os.getpgid(self.detector_process.pid), signal.SIGKILL)
                self.detector_process.wait()
                
            except Exception as e:
                self.get_logger().error(f'Error stopping detector: {e}')
            
            finally:
                self.detector_process = None
    
    def publish_status(self):
        """Publish current status to terminal (timer callback)."""
        status = self.state_manager.get_status_dict()
        
        # Print status update
        if self.state_manager.state == ExperimentState.RUNNING:
            exp = self.state_manager.current_experiment
            if exp:
                self.get_logger().info(
                    f'[RUNNING] {exp} | '
                    f'Progress: {self.current_experiment_index + 1}/{len(self.experiment_queue)} '
                    f'({status["progress_percentage"]:.1f}%)',
                    throttle_duration_sec=10.0
                )
        elif self.state_manager.state == ExperimentState.ERROR:
            self.get_logger().error(
                f'[ERROR] {self.state_manager.error_message} | Press R to retry, S to skip',
                throttle_duration_sec=5.0
            )
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.get_logger().info('Shutting down orchestrator...')
        self._stop_detector_process()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = OrchestratorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
