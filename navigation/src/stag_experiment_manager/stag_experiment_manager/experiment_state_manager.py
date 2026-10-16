#!/usr/bin/env python3
"""
Experiment State Manager

Manages experiment states and transitions for the orchestration system.
"""

from enum import Enum
from typing import Optional, Dict, Any
from dataclasses import dataclass


class ExperimentState(Enum):
    """
    States for experiment execution lifecycle.
    """
    IDLE = "idle"                 # System not started or fully stopped
    READY = "ready"               # Ready to start next experiment
    RUNNING = "running"           # Experiment currently running
    PAUSED = "paused"             # Entire series paused
    COMPLETED = "completed"       # All experiments completed
    ERROR = "error"               # Error occurred, waiting for user action


@dataclass
class ExperimentInfo:
    """
    Information about current experiment.
    """
    marker: str
    experiment_type: str
    gait_type: str
    attempt: int
    total_experiments: int
    completed_count: int
    
    @property
    def progress_percentage(self) -> float:
        """Calculate progress percentage."""
        if self.total_experiments == 0:
            return 100.0
        return (self.completed_count / self.total_experiments) * 100.0
    
    def __str__(self) -> str:
        """String representation."""
        return f"{self.marker}_{self.experiment_type}_{self.gait_type}_{self.attempt}"


class ExperimentStateManager:
    """
    Manages experiment state machine and transitions.
    """
    
    def __init__(self):
        """Initialize state manager."""
        self.state = ExperimentState.IDLE
        self.current_experiment: Optional[ExperimentInfo] = None
        self.error_message: Optional[str] = None
        self.state_history = [ExperimentState.IDLE]
    
    def can_transition(self, from_state: ExperimentState, to_state: ExperimentState) -> bool:
        """
        Check if state transition is valid.
        
        Valid transitions:
        - IDLE -> READY: Initial start
        - READY -> RUNNING: Start experiment
        - RUNNING -> READY: Stop experiment and ready for next
        - RUNNING -> ERROR: Error occurred
        - RUNNING -> COMPLETED: Last experiment finished
        - READY -> PAUSED: Pause series
        - PAUSED -> READY: Resume series
        - ERROR -> READY: Retry after error resolution
        - ERROR -> COMPLETED: Skip after error
        - ANY -> IDLE: Emergency stop/reset
        
        Args:
            from_state: Current state
            to_state: Desired state
        
        Returns:
            True if transition is valid
        """
        # Can always reset to IDLE
        if to_state == ExperimentState.IDLE:
            return True
        
        # Define valid transitions
        valid_transitions = {
            ExperimentState.IDLE: [ExperimentState.READY],
            ExperimentState.READY: [ExperimentState.RUNNING, ExperimentState.PAUSED, ExperimentState.COMPLETED],
            ExperimentState.RUNNING: [ExperimentState.READY, ExperimentState.ERROR, ExperimentState.COMPLETED],
            ExperimentState.PAUSED: [ExperimentState.READY],
            ExperimentState.ERROR: [ExperimentState.READY, ExperimentState.COMPLETED],
            ExperimentState.COMPLETED: [ExperimentState.IDLE, ExperimentState.READY],  # Can restart
        }
        
        return to_state in valid_transitions.get(from_state, [])
    
    def transition_to(self, new_state: ExperimentState, reason: str = "") -> bool:
        """
        Transition to a new state.
        
        Args:
            new_state: State to transition to
            reason: Reason for transition (for logging)
        
        Returns:
            True if transition was successful
        """
        if not self.can_transition(self.state, new_state):
            return False
        
        old_state = self.state
        self.state = new_state
        self.state_history.append(new_state)
        
        # Clear error message when leaving ERROR state
        if old_state == ExperimentState.ERROR and new_state != ExperimentState.ERROR:
            self.error_message = None
        
        return True
    
    def start_experiment(self, experiment: ExperimentInfo) -> bool:
        """
        Start a new experiment.
        
        Args:
            experiment: Experiment information
        
        Returns:
            True if started successfully
        """
        if not self.can_transition(self.state, ExperimentState.RUNNING):
            return False
        
        self.current_experiment = experiment
        return self.transition_to(ExperimentState.RUNNING, f"Starting {experiment}")
    
    def stop_experiment(self) -> bool:
        """
        Stop current experiment and return to READY state.
        
        Returns:
            True if stopped successfully
        """
        if self.state != ExperimentState.RUNNING:
            return False
        
        return self.transition_to(ExperimentState.READY, "Experiment stopped by user")
    
    def mark_completed(self) -> bool:
        """
        Mark current experiment as completed.
        
        Returns:
            True if marked successfully
        """
        if self.current_experiment:
            self.current_experiment.completed_count += 1
        
        # Check if all experiments are done
        if (self.current_experiment and 
            self.current_experiment.completed_count >= self.current_experiment.total_experiments):
            return self.transition_to(ExperimentState.COMPLETED, "All experiments completed")
        
        return self.transition_to(ExperimentState.READY, "Experiment completed, ready for next")
    
    def set_error(self, error_msg: str) -> bool:
        """
        Set error state with message.
        
        Args:
            error_msg: Error message
        
        Returns:
            True if error state set
        """
        self.error_message = error_msg
        return self.transition_to(ExperimentState.ERROR, f"Error: {error_msg}")
    
    def pause_series(self) -> bool:
        """
        Pause the entire experiment series.
        
        Returns:
            True if paused successfully
        """
        if self.state != ExperimentState.READY:
            return False
        
        return self.transition_to(ExperimentState.PAUSED, "Series paused by user")
    
    def resume_series(self) -> bool:
        """
        Resume experiment series from pause.
        
        Returns:
            True if resumed successfully
        """
        if self.state != ExperimentState.PAUSED:
            return False
        
        return self.transition_to(ExperimentState.READY, "Series resumed")
    
    def reset(self) -> bool:
        """
        Reset to IDLE state.
        
        Returns:
            Always True
        """
        self.current_experiment = None
        self.error_message = None
        return self.transition_to(ExperimentState.IDLE, "System reset")
    
    def get_status_dict(self) -> Dict[str, Any]:
        """
        Get current status as dictionary.
        
        Returns:
            Dictionary with status information
        """
        status = {
            "state": self.state.value,
            "error_message": self.error_message,
        }
        
        if self.current_experiment:
            status.update({
                "current_marker": self.current_experiment.marker,
                "current_experiment": self.current_experiment.experiment_type,
                "current_gait": self.current_experiment.gait_type,
                "current_attempt": self.current_experiment.attempt,
                "total_experiments": self.current_experiment.total_experiments,
                "completed_experiments": self.current_experiment.completed_count,
                "progress_percentage": self.current_experiment.progress_percentage,
            })
        else:
            status.update({
                "current_marker": None,
                "current_experiment": None,
                "current_gait": None,
                "current_attempt": None,
                "total_experiments": 0,
                "completed_experiments": 0,
                "progress_percentage": 0.0,
            })
        
        return status
    
    def get_state_history(self) -> list:
        """Get history of state transitions."""
        return [s.value for s in self.state_history]
