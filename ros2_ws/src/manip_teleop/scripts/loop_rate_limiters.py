
#!/usr/bin/env python3

import time
import threading
from typing import Optional

class RateLimiter:
    """A utility class to limit loop frequency."""
    
    def __init__(self, frequency: float, warn: bool = True):
        """
        Initialize a RateLimiter to control loop frequency.
        
        Args:
            frequency: Target frequency in Hz
            warn: Whether to print warnings when rate cannot be maintained
        """
        self.target_dt = 1.0 / frequency
        self.warn = warn
        self.last_time = time.time()
        self.dt = self.target_dt
        
    def sleep(self) -> None:
        """
        Sleep to maintain the target frequency.
        
        This method will calculate how long to sleep based on the time
        since the last call to sleep() and the target frequency.
        """
        current_time = time.time()
        elapsed = current_time - self.last_time
        self.dt = elapsed
        
        # Calculate sleep time
        sleep_time = max(0.0, self.target_dt - elapsed)
        
        # Print warning if we're running slower than target rate
        if self.warn and elapsed > self.target_dt * 1.1:
            print(f"Warning: Loop running slower than target rate. "
                  f"Elapsed time: {elapsed:.4f}s, Target: {self.target_dt:.4f}s")
        
        # Sleep to maintain rate
        if sleep_time > 0:
            time.sleep(sleep_time)
            
        self.last_time = time.time()
        
    def get_frequency(self) -> float:
        """
        Get the current actual frequency in Hz.
        
        Returns:
            The current frequency based on actual elapsed time.
        """
        return 1.0 / self.dt
