"""
High Occlusion (HO) Marker Generation Module

Generates high-occlusion resistant markers by rotating and combining
the same marker into different RGB channels with different orientations.
"""

import os
import random
from PIL import Image

from stag_ros2_enhanced.marker_generators import utils


def combine_rgb_channels(red_file, green_file, blue_file, output_file):
    """
    Combine three marker images with rotation into RGB channels.
    
    Args:
        red_file: Path to marker for red channel (0° rotation)
        green_file: Path to marker for green channel (-90° rotation)
        blue_file: Path to marker for blue channel (-180° rotation)
        output_file: Path to save combined marker
    """
    # Load the images
    red_img = Image.open(red_file)
    green_img = Image.open(green_file)
    blue_img = Image.open(blue_file)

    # Convert to binary
    r = utils.convert_to_binary(red_img)
    g = utils.convert_to_binary(green_img)
    b = utils.convert_to_binary(blue_img)

    # Rotate images for different orientations
    r = r.rotate(0, expand=True)
    g = g.rotate(-90, expand=True)
    b = b.rotate(-180, expand=True)

    # Create a new image from the R, G, B channels
    new_image = Image.merge("RGB", (r, g, b))
    new_image.save(output_file)


def generate_high_occlusion_markers(input_set, input_dir, output_dir, sample_size=-1):
    """
    Generate high-occlusion resistant markers from HD markers.
    
    Args:
        input_set: Input marker set name (e.g., 'HD23')
        input_dir: Directory containing input HD marker files
        output_dir: Directory to save generated HO markers
        sample_size: Number of markers to generate (-1 for all)
    """
    # Identify input files
    m = input_set.replace('HD', '')
    files = utils.list_files(input_dir)

    # Sample if requested
    if sample_size > 0:
        files = random.sample(files, sample_size)

    print(f"Generating {len(files)} HO markers")

    # Combine channels with rotation
    for f in files:
        total = len(str(len(files)))
        uuid = f.replace('.png', '')
        input_file = f"{input_dir}/{f}"

        output_file = f"{output_dir}/HO{m}-ID{str(uuid).zfill(total)}.png"
        combine_rgb_channels(
            red_file=input_file,
            green_file=input_file,
            blue_file=input_file,
            output_file=output_file
        )


if __name__ == '__main__':
    import os
    input_set = 'HD23'
    input_dir = f"{os.getenv('HOME')}/STag-Markers/standard/HD23/"
    output_dir = f"{os.getenv('HOME')}/STag-Markers/high-occlusion/HO23/"

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    generate_high_occlusion_markers(input_set, input_dir, output_dir)

    print('\nGenerated markers.\n')
