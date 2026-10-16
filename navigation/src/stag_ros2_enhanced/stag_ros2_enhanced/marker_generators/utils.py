"""
Utility Functions for Marker Generation

Common functions used across different marker generation modules.
"""

import os
from PIL import Image, ImageDraw


def convert_to_binary(img, threshold=128):
    """
    Convert image to binary (black and white) using threshold.
    
    Args:
        img: PIL Image object
        threshold: Threshold value for binarization (default: 128)
        
    Returns:
        Binary image in grayscale mode
    """
    # Convert the image to grayscale to simplify thresholding
    grayscale = img.convert('L')

    # Apply the threshold to convert the image to binary
    binary = grayscale.point(lambda x: 255 if x > threshold else 0, '1')

    # Convert the binary "1" mode image back to "L" mode (grayscale)
    binary_l = binary.convert('L')

    return binary_l


def list_files(directory):
    """
    List all PNG files in a directory.
    
    Args:
        directory: Path to directory
        
    Returns:
        Sorted list of PNG filenames
    """
    # Get a list of all entries in the directory
    entries = os.listdir(directory)

    # Filter out directories, only keep PNG files
    files = [
        entry for entry in entries
        if os.path.isfile(os.path.join(directory, entry)) and entry.endswith('.png')
    ]
    
    return sorted(files)


def draw_occlusion(image, x, y, radius, fill_value):
    """
    Draw a circular occlusion on the image.
    
    Args:
        image: PIL Image object
        x: X coordinate of circle center
        y: Y coordinate of circle center
        radius: Circle radius
        fill_value: Fill color/value
        
    Returns:
        Modified image
    """
    # Initialize drawing context
    draw = ImageDraw.Draw(image)

    # Define bounding box
    bounding_box = [(x - radius, y - radius), (x + radius, y + radius)]

    # Draw the circle
    draw.ellipse(bounding_box, fill=fill_value)

    return image
