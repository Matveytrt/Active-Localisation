#!/usr/bin/env python3
"""
Keyboard Control Node for Experiment Orchestration

Reads keyboard commands and publishes them to the control topic.
Runs in a separate terminal; publishes to /experiment/control.
"""

import sys
import termios
import tty
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class KeyboardControlNode(Node):
    """
    Node that reads keyboard input and publishes control commands.
    
    Key mappings:
    - SPACE: Start/Stop current experiment
    - N: Next experiment (skip current)
    - R: Retry current experiment
    - P: Pause entire series
    - Q: Quit entire series
    - S: Skip current experiment
    - H: Show help
    """
    
    # Control command constants
    CMD_START_STOP = "start_stop"
    CMD_NEXT = "next"
    CMD_RETRY = "retry"
    CMD_PAUSE = "pause"
    CMD_QUIT = "quit"
    CMD_SKIP = "skip"
    
    def __init__(self):
        super().__init__('keyboard_control')
        
        self.control_pub = self.create_publisher(
            String,
            '/experiment/control',
            10
        )
        
        # Store terminal settings for restoration
        self.terminal_settings: Optional[list] = None
        
        # Timer for keyboard reading (non-blocking)
        self.create_timer(0.05, self.read_keyboard)  # 50ms = 20Hz
        
        self.get_logger().info('Keyboard Control Node started')
        self.print_help()
        
    def print_help(self):
        """Print keyboard command help."""
        help_text = """
╔══════════════════════════════════════════════════════════════╗
║          EXPERIMENT ORCHESTRATOR - KEYBOARD CONTROLS          ║
╠══════════════════════════════════════════════════════════════╣
║  SPACE  │  Start/Stop current experiment                     ║
║  N      │  Next - Skip to next experiment                    ║
║  R      │  Retry - Repeat current experiment                 ║
║  P      │  Pause - Pause entire experiment series            ║
║  Q      │  Quit - Stop entire series and exit               ║
║  S      │  Skip - Skip current experiment after error        ║
║  H      │  Help - Show this help message                     ║
╚══════════════════════════════════════════════════════════════╝
"""
        print(help_text)
        sys.stdout.flush()
    
    def setup_terminal(self):
        """Set terminal to raw mode for character-by-character input."""
        if sys.stdin.isatty():
            self.terminal_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
    
    def restore_terminal(self):
        """Restore terminal to original settings."""
        if self.terminal_settings and sys.stdin.isatty():
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.terminal_settings)
    
    def get_key(self) -> Optional[str]:
        """
        Get a single keypress (non-blocking).
        
        Returns:
            Key character or None if no key pressed
        """
        if sys.stdin.isatty():
            import select
            # Check if there's input available
            if select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.read(1)
        return None
    
    def read_keyboard(self):
        """Read keyboard input and publish commands (timer callback)."""
        key = self.get_key()
        
        if key is None:
            return
        
        # Convert to lowercase for case-insensitive matching
        key = key.lower()
        
        command = None
        command_name = None
        
        # Map keys to commands
        if key == ' ':  # Space
            command = self.CMD_START_STOP
            command_name = "START/STOP"
        elif key == 'n':
            command = self.CMD_NEXT
            command_name = "NEXT"
        elif key == 'r':
            command = self.CMD_RETRY
            command_name = "RETRY"
        elif key == 'p':
            command = self.CMD_PAUSE
            command_name = "PAUSE"
        elif key == 'q':
            command = self.CMD_QUIT
            command_name = "QUIT"
        elif key == 's':
            command = self.CMD_SKIP
            command_name = "SKIP"
        elif key == 'h':
            self.print_help()
            return
        elif key == '\x03':  # Ctrl+C
            self.get_logger().info('Ctrl+C detected, shutting down...')
            rclpy.shutdown()
            return
        
        # Publish command if recognized
        if command:
            msg = String()
            msg.data = command
            self.control_pub.publish(msg)
            
            self.get_logger().info(f'Command sent: {command_name}')
            print(f'>>> {command_name} <<<')
            sys.stdout.flush()
    
    def destroy_node(self):
        """Cleanup when node is destroyed."""
        self.restore_terminal()
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    node = KeyboardControlNode()
    
    try:
        # Setup terminal for raw input
        node.setup_terminal()
        
        # Spin the node
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    
    finally:
        # Cleanup
        node.restore_terminal()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
