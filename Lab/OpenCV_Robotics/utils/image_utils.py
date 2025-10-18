#!/usr/bin/env python3
"""
Common utility functions for OpenCV robotics projects.
"""

import cv2
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import time
import json

class ImageUtils:
    """Utility class for common image operations."""
    
    @staticmethod
    def load_image(image_path, color_mode='color'):
        """
        Load an image with error handling.
        
        Args:
            image_path (str): Path to image file
            color_mode (str): 'color', 'grayscale', or 'unchanged'
        
        Returns:
            numpy.ndarray or None: Loaded image or None if failed
        """
        if color_mode == 'grayscale':
            image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        elif color_mode == 'unchanged':
            image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        else:
            image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        
        if image is None:
            print(f"Error: Could not load image from {image_path}")
        
        return image
    
    @staticmethod
    def save_image(image, filepath, quality=95):
        """
        Save an image with compression settings.
        
        Args:
            image: Image to save
            filepath: Output file path
            quality: JPEG quality (0-100)
        """
        filepath = Path(filepath)
        filepath.parent.mkdir(parents=True, exist_ok=True)
        
        if filepath.suffix.lower() == '.jpg' or filepath.suffix.lower() == '.jpeg':
            cv2.imwrite(str(filepath), image, [cv2.IMWRITE_JPEG_QUALITY, quality])
        else:
            cv2.imwrite(str(filepath), image)
    
    @staticmethod
    def resize_image(image, width=None, height=None, maintain_aspect=True):
        """
        Resize an image while optionally maintaining aspect ratio.
        
        Args:
            image: Input image
            width: Target width
            height: Target height
            maintain_aspect: Whether to maintain aspect ratio
        
        Returns:
            numpy.ndarray: Resized image
        """
        h, w = image.shape[:2]
        
        if width is None and height is None:
            return image
        
        if maintain_aspect:
            if width is None:
                # Calculate width based on height
                ratio = height / h
                width = int(w * ratio)
            elif height is None:
                # Calculate height based on width
                ratio = width / w
                height = int(h * ratio)
            else:
                # Use the smaller ratio to fit within bounds
                ratio = min(width / w, height / h)
                width = int(w * ratio)
                height = int(h * ratio)
        
        return cv2.resize(image, (width, height), interpolation=cv2.INTER_AREA)
    
    @staticmethod
    def create_grid_display(images, titles=None, rows=None, cols=None, figsize=(15, 10)):
        """
        Display multiple images in a grid layout.
        
        Args:
            images: List of images to display
            titles: List of titles for each image
            rows, cols: Grid dimensions (auto-calculated if None)
            figsize: Figure size tuple
        """
        n_images = len(images)
        
        if rows is None and cols is None:
            cols = int(np.ceil(np.sqrt(n_images)))
            rows = int(np.ceil(n_images / cols))
        elif rows is None:
            rows = int(np.ceil(n_images / cols))
        elif cols is None:
            cols = int(np.ceil(n_images / rows))
        
        plt.figure(figsize=figsize)
        
        for i, img in enumerate(images):
            if i >= rows * cols:
                break
                
            plt.subplot(rows, cols, i + 1)
            
            if len(img.shape) == 3:
                if img.shape[2] == 3:  # BGR to RGB
                    img_display = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                else:
                    img_display = img
                plt.imshow(img_display)
            else:
                plt.imshow(img, cmap='gray')
            
            if titles and i < len(titles):
                plt.title(titles[i])
            
            plt.axis('off')
        
        plt.tight_layout()
        plt.show()

class PerformanceTimer:
    """Context manager for timing code execution."""
    
    def __init__(self, description="Operation"):
        self.description = description
        self.start_time = None
    
    def __enter__(self):
        self.start_time = time.time()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        elapsed_time = time.time() - self.start_time
        print(f"{self.description} took {elapsed_time:.4f} seconds")

class ROI:
    """Region of Interest utilities for robotics applications."""
    
    @staticmethod
    def select_roi_interactive(image, window_name="Select ROI"):
        """
        Interactively select a region of interest.
        
        Args:
            image: Input image
            window_name: Window name for selection
        
        Returns:
            tuple: (x, y, w, h) coordinates of selected ROI
        """
        roi = cv2.selectROI(window_name, image, False)
        cv2.destroyWindow(window_name)
        return roi
    
    @staticmethod
    def extract_roi(image, roi):
        """
        Extract region of interest from image.
        
        Args:
            image: Input image
            roi: Tuple (x, y, w, h) defining the ROI
        
        Returns:
            numpy.ndarray: Extracted ROI
        """
        x, y, w, h = roi
        return image[y:y+h, x:x+w]
    
    @staticmethod
    def draw_roi(image, roi, color=(0, 255, 0), thickness=2):
        """
        Draw ROI rectangle on image.
        
        Args:
            image: Input image
            roi: Tuple (x, y, w, h) defining the ROI
            color: Rectangle color
            thickness: Line thickness
        
        Returns:
            numpy.ndarray: Image with drawn ROI
        """
        result = image.copy()
        x, y, w, h = roi
        cv2.rectangle(result, (x, y), (x + w, y + h), color, thickness)
        return result

class ColorSpaceConverter:
    """Utilities for color space conversions."""
    
    @staticmethod
    def convert_colorspace(image, conversion):
        """
        Convert image between color spaces.
        
        Args:
            image: Input image
            conversion: OpenCV color conversion code
        
        Returns:
            numpy.ndarray: Converted image
        """
        return cv2.cvtColor(image, conversion)
    
    @staticmethod
    def bgr_to_rgb(image):
        """Convert BGR to RGB."""
        return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    
    @staticmethod
    def bgr_to_hsv(image):
        """Convert BGR to HSV."""
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    @staticmethod
    def bgr_to_lab(image):
        """Convert BGR to LAB."""
        return cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    
    @staticmethod
    def create_color_range_mask(image, lower_bound, upper_bound, colorspace='HSV'):
        """
        Create a mask for pixels within a color range.
        
        Args:
            image: Input image (BGR)
            lower_bound: Lower color bound
            upper_bound: Upper color bound
            colorspace: Color space for thresholding
        
        Returns:
            numpy.ndarray: Binary mask
        """
        if colorspace == 'HSV':
            converted = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        elif colorspace == 'LAB':
            converted = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
        else:
            converted = image
        
        mask = cv2.inRange(converted, lower_bound, upper_bound)
        return mask

class DatasetManager:
    """Manage datasets and sample data for learning."""
    
    def __init__(self, base_path):
        self.base_path = Path(base_path)
        self.base_path.mkdir(exist_ok=True)
    
    def save_sample_data(self, data, filename, metadata=None):
        """
        Save sample data with optional metadata.
        
        Args:
            data: Data to save (image, array, etc.)
            filename: Output filename
            metadata: Optional metadata dictionary
        """
        filepath = self.base_path / filename
        
        if isinstance(data, np.ndarray) and len(data.shape) >= 2:
            # Save as image
            cv2.imwrite(str(filepath), data)
        else:
            # Save as numpy array
            np.save(str(filepath.with_suffix('.npy')), data)
        
        if metadata:
            metadata_path = filepath.with_suffix('.json')
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
    
    def load_sample_data(self, filename):
        """
        Load sample data and metadata.
        
        Args:
            filename: File to load
        
        Returns:
            tuple: (data, metadata)
        """
        filepath = self.base_path / filename
        
        # Try to load as image first
        data = cv2.imread(str(filepath))
        
        if data is None:
            # Try to load as numpy array
            npy_path = filepath.with_suffix('.npy')
            if npy_path.exists():
                data = np.load(str(npy_path))
        
        # Load metadata if available
        metadata_path = filepath.with_suffix('.json')
        metadata = None
        if metadata_path.exists():
            with open(metadata_path, 'r') as f:
                metadata = json.load(f)
        
        return data, metadata
    
    def list_samples(self, extension=None):
        """
        List available sample files.
        
        Args:
            extension: Filter by file extension
        
        Returns:
            list: List of available files
        """
        if extension:
            return list(self.base_path.glob(f"*.{extension}"))
        else:
            return list(self.base_path.glob("*"))

def setup_matplotlib():
    """Setup matplotlib for better image display."""
    plt.rcParams['figure.figsize'] = (12, 8)
    plt.rcParams['image.cmap'] = 'gray'
    plt.rcParams['image.interpolation'] = 'nearest'

def print_opencv_info():
    """Print OpenCV version and build information."""
    print(f"OpenCV Version: {cv2.__version__}")
    print(f"OpenCV Build Info:")
    print(cv2.getBuildInformation())

if __name__ == "__main__":
    # Demo the utilities
    print("OpenCV Robotics Utilities Demo")
    print("=" * 40)
    
    setup_matplotlib()
    print_opencv_info()
    
    # Create a sample image for testing
    test_image = np.zeros((300, 400, 3), dtype=np.uint8)
    cv2.rectangle(test_image, (50, 50), (150, 150), (255, 100, 100), -1)
    cv2.circle(test_image, (300, 100), 50, (100, 255, 100), -1)
    
    # Demo utilities
    print("\nTesting ImageUtils...")
    resized = ImageUtils.resize_image(test_image, width=200)
    print(f"Original size: {test_image.shape[:2]}")
    print(f"Resized size: {resized.shape[:2]}")
    
    print("\nTesting DatasetManager...")
    dm = DatasetManager("/tmp/opencv_test")
    dm.save_sample_data(test_image, "test_image.png", {"type": "synthetic", "shapes": 2})
    
    print("\nUtilities loaded successfully!")