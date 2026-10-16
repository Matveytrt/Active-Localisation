"""
High Capacity (HC) Marker Generation Module

Generates high-capacity markers by combining RGB channels from standard HD markers.
Each color channel can encode a different marker ID, allowing for much higher
marker density in the same physical space.
"""

import os
from PIL import Image

from stag_ros2_enhanced.marker_generators import utils


def combine_rgb_channels(red_file, green_file, blue_file, output_file):
    """
    Combine three separate marker images into RGB channels.
    
    Args:
        red_file: Path to marker for red channel
        green_file: Path to marker for green channel
        blue_file: Path to marker for blue channel
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

    # Create a new image from the R, G, B channels
    new_image = Image.merge("RGB", (r, g, b))
    new_image.save(output_file)


def generate_high_capacity_markers(input_set, input_dir, output_dir):
    """
    Generate high-capacity markers from a set of HD markers.
    
    Args:
        input_set: Input marker set name (e.g., 'HD23')
        input_dir: Directory containing input HD marker files
        output_dir: Directory to save generated HC markers
    """
    # Load directory
    files = utils.list_files(input_dir)
    print(f"Found {len(files)} input markers")

    # Get hamming distance sizes
    hd_r, hd_g, hd_b = len(files)**0, len(files)**1, len(files)**2
    m = input_set.replace('HD', '')

    # Generate markers
    for b_i, b_f in enumerate(files):
        for g_i, g_f in enumerate(files):
            for r_i, r_f in enumerate(files):

                # Calculate marker ID
                total = len(str(hd_r * hd_g * hd_b))
                uuid = (r_i * hd_r) + (g_i * hd_g) + (b_i * hd_b)
                print(f'M{uuid} | Format(R{r_i}-G{g_i}-B{b_i})')

                # Combine channels
                output_file = f"{output_dir}/HC{m}-{str(uuid).zfill(total)}.png"
                combine_rgb_channels(
                    f"{input_dir}/{r_f}",
                    f"{input_dir}/{g_f}",
                    f"{input_dir}/{b_f}",
                    output_file
                )


if __name__ == '__main__':
    import os
    input_set = 'HD23'
    input_dir = f"{os.getenv('HOME')}/STag-Markers/standard/HD23/"
    output_dir = f"{os.getenv('HOME')}/STag-Markers/high-capacity/HC23/"

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    generate_high_capacity_markers(input_set, input_dir, output_dir)

    print('\nGenerated markers.\n')
