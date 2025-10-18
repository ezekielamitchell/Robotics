# OpenCV for Robotics Learning Project

A comprehensive learning project following a structured OpenCV course curriculum for robotics applications.
ref. Wood, K. (2025). [OpenCV Python Course — Learn Computer Vision and AI](https://youtu.be/TMqH2fYhxh0?si=4MjWietPnzh6MBjr)

## Project Structure

```
OpenCV_Robotics/
├── 01_basic/                 # Basic OpenCV concepts
│   ├── 00_introduction/
│   ├── 01_installation/
│   ├── 02_images/
│   ├── 03_videos/
│   ├── 04_pixels/
│   ├── 05_color_channels/
│   ├── 06_grayscale/
│   ├── 07_hsv_color/
│   ├── 08_image_resizing/
│   ├── 09_image_histogram/
│   ├── 10_convolution/
│   ├── 11_filtering/
│   └── 12_thresholding/
├── 02_advanced/              # Advanced OpenCV techniques
│   ├── 01_image_gradient/
│   ├── 02_canny_edge/
│   ├── 03_line_detection/
│   ├── 04_harris_corner/
│   ├── 05_sift_features/
│   ├── 06_optical_flow/
│   ├── 07_camera_calibration/
│   ├── 08_pose_estimation/
│   └── 09_depth_estimation/
├── datasets/                 # Sample images and test data
├── utils/                    # Common utilities and helper functions
├── experiments/              # Experimental code and prototypes
├── docs/                     # Documentation and tutorials
├── tests/                    # Unit tests and validation scripts
└── requirements.txt          # Python dependencies
```

## Course Curriculum

### 📚 Basic (01_basic/)

| Time | Topic | Description |
|------|-------|-------------|
| 0:00 | Introduction | Course overview and setup |
| 1:15 | Installing OpenCV Python in VS Code | Development environment setup |
| 2:35 | What are Images? | Understanding digital images |
| 6:23 | Read and Write Images | Basic image I/O operations |
| 13:38 | Read and Write Videos | Video processing fundamentals |
| 24:59 | Read and Write Pixels | Pixel-level image manipulation |
| 36:01 | RGB Color Channels | Understanding color spaces |
| 51:41 | Grayscale | Converting to grayscale |
| 56:13 | HSV Color | HSV color space applications |
| 1:06:56 | Image Resizing | Scaling and resizing techniques |
| 1:15:13 | Image Histogram | Analyzing image intensity distributions |
| 1:25:19 | 2D Convolution | Understanding convolution operations |
| 1:31:08 | Average Filtering | Smoothing filters |
| 1:36:10 | Median Filtering | Noise reduction techniques |
| 1:40:32 | Gaussian Filtering | Blur and smoothing |
| 1:50:06 | Image Thresholding | Binary image creation |

### 🚀 Advanced (02_advanced/)

| Time | Topic | Description |
|------|-------|-------------|
| 2:00:46 | Image Gradient | Edge detection fundamentals |
| 2:07:50 | Canny Edge Detection | Advanced edge detection |
| 2:15:42 | Line Detection with Hough Line Transform | Geometric shape detection |
| 2:28:02 | Harris Corner Detection | Corner detection algorithms |
| 2:36:44 | SIFT Feature Detection | Scale-invariant feature detection |
| 2:42:50 | Optical Flow Object Tracking | Motion tracking techniques |
| 2:53:15 | Camera Calibration | Camera parameter estimation |
| 3:07:18 | Pose Estimation | 3D pose from 2D images |
| 3:15:28 | Depth Estimation using Depth Map | 3D depth perception |

## Getting Started

1. **Install dependencies:**
   ```bash
   pip install -r requirements.txt
   ```

2. **Start with Basic concepts:**
   ```bash
   cd 01_basic/00_introduction
   python introduction.py
   ```

3. **Follow the curriculum sequentially** - Each module builds on the previous ones

4. **Use the timestamps** to reference specific topics from the course video

## Learning Path

### Phase 1: Fundamentals (01_basic/)
- Master basic image operations and manipulations
- Understand color spaces and pixel operations
- Learn filtering and enhancement techniques
- Practice with image I/O and video processing

### Phase 2: Advanced Techniques (02_advanced/)
- Implement feature detection algorithms
- Work with camera calibration and 3D vision
- Build object tracking systems
- Create depth perception applications

### Phase 3: Robotics Integration
- Apply learned techniques to robotics scenarios
- Integrate with robot sensors and actuators
- Develop real-time vision processing pipelines

