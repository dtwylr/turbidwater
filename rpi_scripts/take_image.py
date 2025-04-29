import os
import time
import cv2
import numpy as np
from picamera2 import Picamera2
import RPi.GPIO as GPIO
from datetime import datetime
from PIL import Image
import sys
import select
import re

class TurbidWaterCapture:
    def __init__(
        self, 
        button_pin=17, 
        led_pin=22, 
        crop_x=10, 
        crop_y=10, 
        burst_count=1, 
        burst_delay=1
    ):
        """
        Initialize the Turbid Water Capture system.
        
        Args:
            button_pin: GPIO pin for capture button
            led_pin: GPIO pin for LED indicator
            crop_x: Pixels to crop from left/right
            crop_y: Pixels to crop from top/bottom
            burst_count: Number of frames to capture in a burst
            burst_delay: Delay between burst frames in milliseconds
        """
        # Capture parameters
        self.CROP_X = crop_x
        self.CROP_Y = crop_y
        self.BURST_COUNT = burst_count
        self.BURST_DELAY = burst_delay
        
        # GPIO Pins
        self.BUTTON_PIN = button_pin
        self.LED_PIN = led_pin
        
        # Create output directory
        self.BASE_DIR = "turbid_water_imgs"
        os.makedirs(self.BASE_DIR, exist_ok=True)
        
        # Initialize cameras
        self.picam2_left = Picamera2(0)  # Left camera (ground truth)
        self.picam2_right = Picamera2(1)  # Right camera (noisy image)
        
        # Setup GPIO
        self._setup_gpio()
        
        # Configure cameras
        self._configure_cameras()
    
    def _setup_gpio(self):
        """Setup GPIO for button and LED"""
        GPIO.setmode(GPIO.BCM)
        
        # Button setup
        GPIO.setup(self.BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(
            self.BUTTON_PIN, 
            GPIO.FALLING, 
            callback=self._button_pressed, 
            bouncetime=300
        )
        
        # LED setup
        GPIO.setup(self.LED_PIN, GPIO.OUT, initial=GPIO.LOW)
    
    def _configure_cameras(self):
        """Configure camera settings"""
        for camera in [self.picam2_left, self.picam2_right]:
            camera.configure(camera.create_still_configuration())
            camera.start()
            camera.set_controls({
                "AwbMode": 1,
                "AeEnable": False,
                "ExposureTime": 8000,  # in microseconds
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
        pattern = re.compile(r'(\d+)_(gt|tw)\.jpg$')

        for filename in os.listdir(folder):
            match = pattern.match(filename)
            if match:
                index = int(match.group(1))
                highest_index = max(highest_index, index)

        return highest_index if highest_index != -1 else 0
    
    def _burst_capture(self, picam):
        """
        Capture a burst of frames and return an averaged (stabilized) image.
        
        Args:
            picam: Picamera2 object to capture from
        
        Returns:
            Averaged frame from burst capture
        """
        frames = []
        for _ in range(self.BURST_COUNT):
            frame = picam.capture_array()
            frames.append(frame.astype(np.float32))
        
        # Average the burst to reduce accidental motion/noise
        avg_frame = np.mean(frames, axis=0).astype(np.uint8)
        return avg_frame
    
    def _crop_image(self, image):
        """
        Crops an image by predefined pixels.
        
        Args:
            image: Input image to crop
        
        Returns:
            Cropped image
        """
        h, w = image.shape[:2]
        return image[self.CROP_Y:h-self.CROP_Y, self.CROP_X:w-self.CROP_X]
    
    def _button_pressed(self, channel):
        """
        Callback for button press event.
        
        Args:
            channel: GPIO channel triggered
        """
        print("Button pressed. Capturing images with burst stabilization...")
        self._capture_images()
    
    def _capture_images(self):
        """
        Capture and save images from both cameras.
        """
        # Activate LED
        GPIO.output(self.LED_PIN, GPIO.HIGH)
        
        # Create a date-based folder
        today_folder = datetime.now().strftime("%m_%d_%Y")
        save_path = os.path.join(self.BASE_DIR, today_folder)
        os.makedirs(save_path, exist_ok=True)

        # Determine the next index
        index = self._get_highest_index(save_path) + 1

        # Capture a burst from each camera for stabilization
        burst_clear = self._burst_capture(self.picam2_right)
        burst_noisy = self._burst_capture(self.picam2_left)

        # Convert to PIL for processing
        img_clear = Image.fromarray(cv2.cvtColor(burst_clear, cv2.COLOR_BGR2RGB))
        img_noisy = Image.fromarray(cv2.cvtColor(burst_noisy, cv2.COLOR_BGR2RGB))
        
        # Resize images
        img_clear_resized = img_clear.resize((256, 256))
        img_noisy_resized = img_noisy.resize((256, 256))

        # Save images with new naming scheme
        img_clear_resized.save(f"{save_path}/{index}_gt.jpg", quality=85)
        img_noisy_resized.save(f"{save_path}/{index}_tw.jpg", quality=85)

        print(f"Saved: {index}_gt.jpg and {index}_tw.jpg in {today_folder}")
        
        # Deactivate LED
        GPIO.output(self.LED_PIN, GPIO.LOW)
    
    def run(self):
        """
        Run the capture system, waiting for button presses.
        """
        try:
            print("Waiting for button press...")
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            print("\nExiting program.")
        finally:
            # Cleanup GPIO
            GPIO.cleanup()

def main():
    """
    Main entry point for the turbid water capture system.
    """
    capture_system = TurbidWaterCapture(
        button_pin=17,     # GPIO pin for button
        led_pin=22,        # GPIO pin for LED
        crop_x=10,         # Pixels to crop from left/right
        crop_y=10,         # Pixels to crop from top/bottom
        burst_count=1,     # Number of frames in burst
        burst_delay=1      # Delay between burst frames
    )
    capture_system.run()

if __name__ == "__main__":
    main()