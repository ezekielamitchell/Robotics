# Data Directory

This directory contains sample data, test images, and output files for the OpenCV Robotics learning project.

## Structure

```
data/
├── sample_image.png      # Sample input image for testing
├── output_img.jpg        # Example output from image processing
└── datasets/             # Downloaded datasets (not in repo)
```

## Usage

### Sample Data

The `sample_image.png` and `output_img.jpg` files are provided for testing basic image operations without requiring external datasets.

### Datasets

For learning exercises that require larger datasets, they should be downloaded separately and placed in appropriate subdirectories:

```bash
# Example: Create datasets directory
mkdir -p datasets

# Download datasets as needed for specific exercises
# See individual lesson READMEs for dataset requirements
```

## Gitignore

Large datasets and generated files should not be committed to the repository. The following are ignored:
- `datasets/` directory
- Large image/video files (configure in .gitignore if needed)
- Temporary processing outputs

## Best Practices

1. **Sample Data**: Keep sample files small (<1MB) for quick testing
2. **Datasets**: Document dataset sources and versions
3. **Outputs**: Clean up generated outputs regularly
4. **Caching**: Use local caching for downloaded datasets

## External Datasets

Common datasets for robotics computer vision:
- COCO (Common Objects in Context)
- ImageNet
- Pascal VOC
- Open Images Dataset
- RoboFlow public datasets

See individual lesson documentation for specific dataset requirements.
