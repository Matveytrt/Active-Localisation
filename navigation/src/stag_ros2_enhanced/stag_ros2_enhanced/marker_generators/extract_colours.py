"""
Color Extraction Utility

Extract specific colors from marker images and create binary masks.
Useful for analyzing multi-color markers.
"""

import sys
from PIL import Image
import numpy as np
from scipy.ndimage import binary_erosion


def extract_color_binary(image_path, target_color, threshold=60, structure=None):
    """
    Extract pixels close to the target color from an image and create a binary image,
    then apply morphological erosion to reduce artifacts.

    Args:
        image_path: Path to the input image
        target_color: The RGB color to match (e.g., (255, 0, 0) for red)
        threshold: Distance threshold to determine 'closeness' to the target color
        structure: Structuring element for morphological operations

    Returns:
        PIL.Image: Binary image after morphological erosion
    """
    # Open image and convert to RGB
    image = Image.open(image_path).convert('RGB')
    
    # Convert image to numpy array
    data = np.array(image)
    
    # Compute distance from target color
    distance = np.linalg.norm(data - target_color, axis=2)
    
    # Create binary mask where distance is less than threshold
    mask = distance < threshold
    
    # Apply morphological erosion to the mask
    eroded_mask = binary_erosion(mask, structure=structure)
    
    # Create binary image: white where mask is True, black elsewhere
    binary_image = np.zeros_like(data[:, :, 0], dtype=np.uint8)
    binary_image[eroded_mask] = 255
    
    # Convert to PIL Image and return
    return Image.fromarray(binary_image)


# Define target colors
COLORS = {
    'red': (255, 0, 0),
    'green': (0, 255, 0),
    'blue': (0, 0, 255),
    'cyan': (0, 255, 255),
    'yellow': (255, 255, 0),
    'purple': (255, 0, 255),
    'white': (255, 255, 255),
    'black': (0, 0, 0)
}


if __name__ == '__main__':
    # Get filepath
    if len(sys.argv) < 2:
        print("Usage: python extract_colours.py input_image.png")
        sys.exit(1)
    
    file = sys.argv[1]

    # Define the structuring element for morphological erosion
    # For example, a 3x3 square
    structuring_element = np.ones((3, 3), dtype=bool)

    # Process the image for each color
    input_image_path = file
    for color_name, color_value in COLORS.items():
        # Adjust threshold for white and black
        if color_name in ['white', 'black']:
            threshold = 80  # Higher threshold for white and black
        else:
            threshold = 60
        
        binary_image = extract_color_binary(
            input_image_path,
            color_value,
            threshold,
            structure=structuring_element
        )
        binary_image.save(f'binary_{color_name}.png')
        print(f"Saved binary_{color_name}.png")
