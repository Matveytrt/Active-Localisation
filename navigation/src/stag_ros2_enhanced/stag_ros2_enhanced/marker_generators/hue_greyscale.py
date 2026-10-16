"""
Hue Greyscale (HG) Marker Generation Module

Generates hue/greyscale markers by applying color shades to standard HD markers.
Supports both chromatic (colored) and greyscale shading variants.
"""

import os
from PIL import Image

from stag_ros2_enhanced.marker_generators import utils


def generate_hue_marker(input_file, output_file, base_colour, detail_colour, threshold=128):
    """
    Generate a hue marker with two specified colors.
    
    Args:
        input_file: Path to input marker image
        output_file: Path to save generated marker
        base_colour: RGB tuple for base color
        detail_colour: RGB tuple for detail color
        threshold: Threshold for binary conversion (default: 128)
    """
    # Load the image and convert to grayscale
    img = Image.open(input_file).convert("L")

    # Apply threshold to create a binary mask
    binary_mask = img.point(lambda x: 255 if x >= threshold else 0).convert('1')

    # Create solid color images for base and detail colors
    base_image = Image.new('RGB', img.size, base_colour)
    detail_image = Image.new('RGB', img.size, detail_colour)

    # Composite the images using the binary mask
    new_image = Image.composite(detail_image, base_image, binary_mask)

    # Save the new image
    new_image.save(output_file)


def generate_greyscale_marker(input_file, output_file, base_shade, detail_shade):
    """
    Generate a greyscale marker with two specified shade values.
    
    Args:
        input_file: Path to input marker image
        output_file: Path to save generated marker
        base_shade: Greyscale value for base (0-255)
        detail_shade: Greyscale value for detail (0-255)
    """
    # Load the image
    img = Image.open(input_file)

    # Convert to binary
    binary = utils.convert_to_binary(img)
    threshold = 128

    # Define a lookup table to map each pixel
    # Pixels below threshold -> base_shade
    # Pixels equal to or above threshold -> detail_shade
    lut = [base_shade if i < threshold else detail_shade for i in range(256)]

    # Apply the lookup table to the image
    new_image = binary.point(lut)

    # Save new image
    new_image.save(output_file)


def generate_hue_greyscale_markers(input_set, input_dir, output_dir, colour=None):
    """
    Generate hue/greyscale markers from HD markers.
    
    Args:
        input_set: Input marker set name (e.g., 'HD23')
        input_dir: Directory containing input HD marker files
        output_dir: Directory to save generated HG markers
        colour: Optional list of (base_colour, detail_colour) tuples for chromatic markers
    """
    if not colour:
        colour = []

    # Load directory
    files = utils.list_files(input_dir)
    print(f"Found {len(files)} input markers")

    # Apply chromatic shades
    if len(colour) > 0:
        shades = colour
        output_dir += "chromatic/"
    # Apply greyscale pairs
    else:
        s1 = sorted(list(range(0, 101, 10)) + list(range(41, 50, 1)) + list(range(51, 60, 1)))
        s2 = [100 - s for s in s1]
        shades = [(int((s1[i] / 100) * 255), int((s2[i] / 100) * 255)) for i in range(len(s1))]

    # Generate markers
    total = 3
    print(f"Generating with shades: {shades}")
    shadetotal = len(str(len(shades)))
    
    for f in files:
        for i, s in enumerate(shades):
            # Calculate marker ID
            base_shade, detail_shade = s[0], s[1]
            
            if type(s[0]) == tuple:
                # Chromatic (colored) markers
                bs = f'{s[0][0]:02X}{s[0][1]:02X}{s[0][2]:02X}'
                ds = f'{s[1][0]:02X}{s[1][1]:02X}{s[1][2]:02X}'
            else:
                # Greyscale markers
                bs, ds = str(base_shade).zfill(total), str(detail_shade).zfill(total)
            
            uuid = f"{f.replace('.png', '')}-{bs}-{ds}.png"
            print(f"Generating: {f} -> shade {i} {s} -> {uuid}")

            # Combine channels
            output_file = f"{output_dir}/HG{input_set}-{uuid}"
            
            if type(s[0]) == tuple:
                generate_hue_marker(f"{input_dir}/{f}", output_file, base_shade, detail_shade)
            else:
                generate_greyscale_marker(f"{input_dir}/{f}", output_file, base_shade, detail_shade)


if __name__ == '__main__':
    import os
    input_set = '23'
    input_dir = f"{os.getenv('HOME')}/STag-Markers/standard/HD{input_set}/"
    output_dir = f"{os.getenv('HOME')}/STag-Markers/hue-greyscale/HG{input_set}/"

    colour = [
        ((255, 0, 0), (0, 0, 0)),
        ((255, 222, 0), (54, 124, 43)),
        ((0, 69, 124), (0, 121, 193))
    ]

    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    if colour and not os.path.exists(output_dir + "chromatic/"):
        os.makedirs(output_dir + "chromatic/")
    
    generate_hue_greyscale_markers(input_set, input_dir, output_dir, colour)

    print('\nGenerated markers.\n')
