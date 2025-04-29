import os
import time
import cv2
import numpy as np
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput
import RPi.GPIO as GPIO
from datetime import datetime
from PIL import Image
import sys
import select
import re

#EXPOSURE_TIME = 20000  # Indoor with poor lighting
#EXPOSURE_TIME = 10000  # Indoor with good to moderate lighting
EXPOSURE_TIME = 800   # Outdoor during cloudy day
#EXPOSURE_TIME = 400   # Outdoor during sunny day?
ANALOGUE_GAIN = 0  # Auto

class TurbidWaterVideoCapture:
    def __init__(
        self, 
        button_pin=17, 
        led_pin=22, 
        video_duration=30  # Default maximum video duration in seconds
    ):
        """
        Initialize the Turbid Water Video Capture system.
        
        Args:
            button_pin: GPIO pin for capture button
            led_pin: GPIO pin for LED indicator
            video_duration: Maximum duration of video in seconds
        """
        # Video parameters
        self.VIDEO_DURATION = video_duration
        
        # GPIO Pins
        self.BUTTON_PIN = button_pin
        self.LED_PIN = led_pin
        
        # Create output directory
        self.BASE_DIR = "turbid_water_videos"
        os.makedirs(self.BASE_DIR, exist_ok=True)
        
        # Initialize cameras
        self.picam2_left = Picamera2(0)  # Left camera (ground truth)
        self.picam2_right = Picamera2(1)  # Right camera (noisy image)
        
        # Initialize encoders
        self.encoder_left = H264Encoder()
        self.encoder_right = H264Encoder()
        
        # Recording state
        self.is_recording = False
        self.recording_start_time = None
        self.current_output_path = None
        
        # Setup GPIO
        self._setup_gpio()
        
        # Configure cameras
        self._configure_cameras()
    
    def _setup_gpio(self):
        """Setup GPIO for button and LED"""
        GPIO.setmode(GPIO.BCM)
        
        # Button setup - use callback for both press and release
        GPIO.setup(self.BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(
            self.BUTTON_PIN, 
            GPIO.FALLING, 
            callback=self._button_handler, 
            bouncetime=300
        )
        
        # LED setup
        GPIO.setup(self.LED_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    def _configure_cameras(self):
        """Configure camera settings for video"""
        for camera in [self.picam2_left, self.picam2_right]:
            # Configure for video
            camera.configure(camera.create_video_configuration(main={"size": (1280, 720)}))
            camera.start()
            camera.set_controls({
                "AwbMode": 1,
                "AeEnable": False,
                "ExposureTime": EXPOSURE_TIME,  # in microseconds
                "AnalogueGain": 0
            })
    
    def _get_highest_index(self, folder):
        """
        Get the highest existing index in the folder.
        
        Args:
            folder: Path to the folder to check
        
        Returns:
            Highest existing index or 0
        """
        highest_index = -1
        pattern = re.compile(r'(\d+)_(gt|tw)\.mp4$')

        for filename in os.listdir(folder):
            match = pattern.match(filename)
            if match:
                index = int(match.group(1))
                highest_index = max(highest_index, index)

        return highest_index if highest_index != -1 else 0
    
    def _button_handler(self, channel):
        """
        Handle button press - toggle between starting and stopping recording
        
        Args:
            channel: GPIO channel triggered
        """
        if not self.is_recording:
            self._start_recording()
        else:
            self._stop_recording()
    
    def _start_recording(self):
        """
        Start recording video from both cameras
        """
        if self.is_recording:
            return
            
        print("Starting video recording...")
        
        # Create a date-based folder
        today_folder = datetime.now().strftime("%m_%d_%Y")
        save_path = os.path.join(self.BASE_DIR, today_folder)
        os.makedirs(save_path, exist_ok=True)

        # Determine the next index
        index = self._get_highest_index(save_path) + 1
        self.current_output_path = save_path
        self.current_index = index
        
        # Setup file outputs
        output_left = FileOutput(f"{save_path}/{index}_gt.mp4")
        output_right = FileOutput(f"{save_path}/{index}_tw.mp4")
        
        # Start encoding
        self.encoder_left.output = output_left
        self.encoder_right.output = output_right
        
        self.picam2_left.start_encoder(self.encoder_left)
        self.picam2_right.start_encoder(self.encoder_right)
        
        # Update state
        self.is_recording = True
        self.recording_start_time = time.time()
        
        # Blink LED to indicate recording started
        self._blink_led(3, 0.2)
        
        # Turn on LED solid for recording
        GPIO.output(self.LED_PIN, GPIO.HIGH)
    
    def _stop_recording(self):
        """
        Stop recording and save videos
        """
        if not self.is_recording:
            return
            
        print("Stopping video recording...")
        
        # Stop encoders
        self.picam2_left.stop_encoder()
        self.picam2_right.stop_encoder()
        
        # Update state
        self.is_recording = False
        elapsed_time = time.time() - self.recording_start_time
        
        # Blink LED to indicate recording stopped
        GPIO.output(self.LED_PIN, GPIO.LOW)
        self._blink_led(2, 0.5)
        
        print(f"Saved: {self.current_index}_gt.mp4 and {self.current_index}_tw.mp4")
        print(f"Recording duration: {elapsed_time:.2f} seconds")
    
    def _blink_led(self, times, interval):
        """
        Blink the LED a specified number of times
        
        Args:
            times: Number of blinks
            interval: Time between blinks in seconds
        """
        for _ in range(times):
            GPIO.output(self.LED_PIN, GPIO.HIGH)
            time.sleep(interval)
            GPIO.output(self.LED_PIN, GPIO.LOW)
            time.sleep(interval)
    
    def _check_duration(self):
        """
        Check if recording duration has exceeded max duration
        """
        if not self.is_recording:
            return
            
        elapsed_time = time.time() - self.recording_start_time
        if elapsed_time > self.VIDEO_DURATION:
            print(f"Maximum recording duration ({self.VIDEO_DURATION}s) reached.")
            self._stop_recording()
    
    def run(self):
        """
        Run the capture system, waiting for button presses.
        """
        try:
            print("Waiting for button press to start recording...")
            while True:
                # Check if we need to stop recording due to maximum duration
                self._check_duration()
                time.sleep(0.1)  # Small sleep to prevent CPU hogging
        except KeyboardInterrupt:
            print("\nExiting program.")
            # Make sure to stop recording if interrupted
            if self.is_recording:
                self._stop_recording()
        finally:
            # Cleanup GPIO
            GPIO.cleanup()

def main():
    """
    Main entry point for the turbid water video capture system.
    """
    video_capture_system = TurbidWaterVideoCapture(
        button_pin=17,        # GPIO pin for button
        led_pin=22,           # GPIO pin for LED
        video_duration=60     # Maximum recording duration in seconds
    )
    video_capture_system.run()

if __name__ == "__main__":
    main()
    