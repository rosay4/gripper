import sys
import tty
import termios
import threading
import select
from collections import deque

def flush_input():
    """Flushes the input buffer of the terminal."""
    # This function is more complex with the new setup,
    # as we need to handle the raw terminal mode.
    # For now, we will rely on the non-blocking read to consume stale input.
    # A more robust implementation might be needed if issues persist.
    if sys.stdin.isatty():
        try:
            # Save current terminal settings
            original_settings = termios.tcgetattr(sys.stdin)
            # Set to non-blocking
            tty.setcbreak(sys.stdin.fileno())
            # Read all available characters
            while select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.read(1)
            # Restore original settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, original_settings)
        except (termios.error, OSError):
            pass # Ignore errors if not in a tty


def input_with_flush(prompt: str):
    flush_input()
    return input(prompt)


class KeyboardCommander:
    def __init__(self, commands):
        self.commands = commands
        self.command_queue = deque(maxlen=1)
        self._thread = None
        self._stop_event = threading.Event()
        self.original_settings = None
        self.fd = None

    def get_instructions(self):
        blue = "\x1b[34m"
        reset = "\x1b[0m"
        lines = ["Press a key to issue a command:"]
        lines.append("  q: Quit")
        for key, details in sorted(self.commands.items()):
            lines.append(f"  {key}: {details['description']}")
        return blue + "\n".join(lines) + reset

    def _run(self):
        if not sys.stdin.isatty():
            print("Not a TTY. Keyboard commander disabled.")
            return
            
        self.fd = sys.stdin.fileno()
        self.original_settings = termios.tcgetattr(self.fd)
        try:
            tty.setcbreak(self.fd)
            while not self._stop_event.is_set():
                # Use select for non-blocking check of stdin
                if select.select([self.fd], [], [], 0.1)[0]:
                    char = sys.stdin.read(1)
                    if char:
                        self.command_queue.append(char)
        finally:
            # Always restore terminal settings
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)

    def pause(self):
        """Temporarily restore original terminal settings."""
        if self.original_settings and self.fd is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)

    def resume(self):
        """Resume cbreak mode for single-key input."""
        if self.fd is not None:
            tty.setcbreak(self.fd)

    def start(self):
        if self._thread is None:
            self._stop_event.clear()
            self._thread = threading.Thread(target=self._run, daemon=True)
            self._thread.start()

    def stop(self):
        if self._thread and self._thread.is_alive():
            self._stop_event.set()
            self._thread.join()
        self._thread = None
        # Restore terminal settings just in case the thread didn't exit cleanly
        if self.original_settings and self.fd is not None:
             termios.tcsetattr(self.fd, termios.TCSADRAIN, self.original_settings)

    def get_command(self):
        try:
            return self.command_queue.popleft()
        except IndexError:
            return None

    def get_callback(self, key):
        return self.commands.get(key, {}).get("callback")