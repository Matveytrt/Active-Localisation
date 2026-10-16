import re
import sys

def extract_markers_simple(sdf_file):
    """Simple regex-based extraction of markers from SDF file."""
    
    with open(sdf_file, 'r') as f:
        content = f.read()
    
    markers = []
    
    # Find all model sections
    model_pattern = r'<model name="stag_[^"]*">(.*?)</model>'
    
    for model_match in re.finditer(model_pattern, content, re.DOTALL):
        model_content = model_match.group(1)
        
        # Extract pose
        pose_match = re.search(r'<pose>([^<]+)</pose>', model_content)
        if not pose_match:
            continue
            
        pose_str = pose_match.group(1).strip()
        pose_parts = pose_str.split()
        if len(pose_parts) < 6:
            continue
            
        x, y, z, roll, pitch, yaw = pose_parts[:6]
        
        # Extract marker ID from material name.
        # Supports any namespace, e.g.:
        #   STagMarker/marker_00219
        #   STagMarkerHD15/marker_00003
        #   STagMarkerHD17/marker_00001
        id_match = re.search(r'<name>[^<]*/marker_(\d+)</name>', model_content)
        if id_match:
            marker_id = id_match.group(1)
        else:
            # Try to extract from model name
            name_match = re.search(r'<model name="stag_[^"]*?(\d+)"', model_match.group(0))
            if name_match:
                marker_id = name_match.group(1)
            else:
                continue
        
        markers.append((int(marker_id), x, y, z, roll, pitch, yaw))
    
    # Sort by ID and format
    markers.sort(key=lambda x: x[0])
    formatted = []
    for marker_id, x, y, z, roll, pitch, yaw in markers:
        formatted.append(f"{marker_id} {x} {y} {z} {roll} {pitch} {yaw}")
    
    return "; ".join(formatted)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python extract_markers.py <sdf_file>")
        sys.exit(1)
    
    result = extract_markers_simple(sys.argv[1])
    if result:
        print(result)
    else:
        print("No markers found.")