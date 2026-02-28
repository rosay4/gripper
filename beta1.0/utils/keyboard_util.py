import sys
import termios

from queue import Queue, Empty
from typing import Callable, Dict, Any, Optional

from pynput import keyboard
from pynput.keyboard import Key, KeyCode


def flush_input():
    """Flushes the standard input buffer (for Linux)."""
    try:
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
    except (termios.error, AttributeError):
        # Not all environments support this (e.g., running in certain IDEs)
        pass


class KeyboardCommander:
    """
    A general-purpose utility to handle keyboard commands in a non-blocking way.
    """

    def __init__(self, commands: Dict[str, Dict[str, Any]]):
        """
        Initializes the commander with a dictionary of commands.
        Args:
            commands: A dictionary where keys are keyboard characters and
                      values are dicts with 'description' and 'callback' keys.
        """
        if "q" in commands:
            raise ValueError("'q' is a reserved key for quitting and cannot be overridden.")
        
        self.commands = commands
        self.command_queue = Queue()
        self.listener = keyboard.Listener(on_press=self._on_press)

    def _on_press(self, key: Key | KeyCode | None):
        if hasattr(key, "char"):
            if key.char in self.commands or key.char == 'q':
                self.command_queue.put(key.char)
    
    def clear_queue(self):
        """Clears all pending commands from the queue."""
        while not self.command_queue.empty():
            try:
                self.command_queue.get_nowait()
            except Empty:
                break

    def start(self):
        """Starts the keyboard listener in a separate thread."""
        self.listener.start()

    def stop(self):
        """Stops the keyboard listener."""
        self.listener.stop()

    def get_command(self) -> Optional[str]:
        """
        Retrieves a command from the queue if available.
        Returns the command character or None if the queue is empty.
        """
        try:
            return self.command_queue.get_nowait()
        except Empty:
            return None

    def get_instructions(self) -> str:
        """Returns a formatted string of available commands."""
        lines = ["\x1b[36m", "Available Commands:"]
        # Sort items for consistent order
        sorted_commands = sorted(self.commands.items())
        for key, details in sorted_commands:
            lines.append(f"  [{key}] {details['description']}")
        lines.append("  [q] Quit")
        lines.append("\x1b[0m")
        return "\n".join(lines)

    def get_callback(self, key: str) -> Optional[Callable]:
        """Returns the callback function for a given command key."""
        return self.commands.get(key, {}).get('callback')