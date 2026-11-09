"""Audit the project's actual SDF world before converting it to USD.

All poses are composed model -> link -> geometry. Unknown geometry or named
relative frames fail explicitly, rather than silently dropping scene objects.
This module does not fabricate detector results or copy a team world/map.
"""
import argparse
import hashlib
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
from prepare_robot import rpy_matrix


def pose_matrix(element):
    pose = element.find('pose')
    matrix = np.eye(4)
    if pose is None:
        return matrix
    if pose.get('relative_to') or pose.get('rotation_format', 'euler_rpy') != 'euler_rpy':
        raise ValueError('Named/alternative SDF pose frames need an explicit resolver')
    values = list(map(float, pose.text.split()))
    if len(values) != 6 or pose.get('degrees', 'false') != 'false':
        raise ValueError('Expected six SDF pose values in meters/radians')
    matrix[:3, :3] = rpy_matrix(*values[3:])
    matrix[:3, 3] = values[:3]
    return matrix


def resolve_uri(package, uri):
    if uri.startswith('model://'):
        path = package/'gazebo_objects'/uri[len('model://'):]
    elif uri.startswith('package://world_and_robot/'):
        path = package/uri[len('package://world_and_robot/'):]
    else:
        raise ValueError('Unsupported project asset URI: '+uri)
    if not path.is_file():
        raise FileNotFoundError(path)
    if not path.resolve().is_relative_to(package.resolve()):
        raise ValueError('Asset escapes the project package')
    return path


def texture_for_material(package, name):
    for script in sorted((package/'gazebo_objects').glob('*/materials/scripts/*')):
        if not script.is_file():
            continue
        for match in re.finditer(r'material\s+([^\s{]+)\s*\{.*?\btexture\s+([^\s}]+)', script.read_text(), re.S):
            if match[1] == name:
                texture = script.parent.parent/'textures'/match[2]
                if not texture.is_file():
                    raise FileNotFoundError(texture)
                return texture
    raise ValueError('No texture for material '+name)


def audit(repo=None):
    repo = Path(repo or Path(__file__).resolve().parents[1])
    package = repo/'gazebo/src/world_and_robot'
    source = package/'worlds/astribot_hd15_hd17_walls.world'
    world = ET.parse(source).getroot().find('world')
    if world.findall('include') or world.findall('frame'):
        raise ValueError('Resolve world includes/frames before conversion')
    result = {'source': str(source.relative_to(repo)).replace('\\', '/'),
              'source_sha256': hashlib.sha256(source.read_bytes()).hexdigest(),
              'models': [], 'collisions': [], 'visuals': [], 'markers': [], 'assets': {}}

    def asset(path):
        name = path.relative_to(repo).as_posix()
        result['assets'][name] = hashlib.sha256(path.read_bytes()).hexdigest()
        return name

    for model in world.findall('model'):
        if model.findall('model') or model.findall('include'):
            raise ValueError('Nested models need an explicit resolver')
        model_name = model.get('name')
        result['models'].append(model_name)
        model_transform = pose_matrix(model)
        if model_name == 'spawn_area_marker':
            result['spawn_xyz'] = model_transform[:3, 3].tolist()
        for link in model.findall('link'):
            link_transform = model_transform @ pose_matrix(link)
            for kind, target in (('collision', 'collisions'), ('visual', 'visuals')):
                for element in link.findall(kind):
                    geometry = element.find('geometry')
                    entry = {'model': model_name, 'link': link.get('name'), 'name': element.get('name'),
                             'transform': (link_transform @ pose_matrix(element)).tolist()}
                    if geometry.find('box') is not None:
                        entry.update(type='box', size=list(map(float, geometry.findtext('box/size').split())))
                    elif geometry.find('plane') is not None:
                        entry.update(type='plane', size=list(map(float, geometry.findtext('plane/size').split())),
                                     normal=list(map(float, geometry.findtext('plane/normal').split())))
                    elif geometry.find('mesh') is not None:
                        entry.update(type='mesh', path=asset(resolve_uri(package, geometry.findtext('mesh/uri'))),
                                     scale=list(map(float, geometry.findtext('mesh/scale', '1 1 1').split())))
                    else:
                        raise ValueError('Unsupported geometry in '+model_name)
                    result[target].append(entry)
                    if kind == 'visual' and model_name.startswith('stag_'):
                        material = element.findtext('material/script/name')
                        match = re.fullmatch(r'STagMarker(HD15|HD17)/marker_(\d+)', material or '')
                        if not match or entry['type'] != 'box':
                            raise ValueError('Unexpected STag visual '+model_name)
                        texture = texture_for_material(package, material)
                        from PIL import Image
                        pixels = np.array(Image.open(texture).convert('RGB'))
                        ys, xs = np.where(np.min(pixels, axis=2) < 100)
                        fraction_x = (int(xs.max())-int(xs.min())+1)/pixels.shape[1]
                        fraction_y = (int(ys.max())-int(ys.min())+1)/pixels.shape[0]
                        result['markers'].append(dict(entry, family=match[1], id=int(match[2]),
                            texture=asset(texture), black_square_fraction=[fraction_x, fraction_y],
                            detected_edge_m=[entry['size'][0]*fraction_x, entry['size'][1]*fraction_y]))
    return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = audit()
    output = args.output or Path(__file__).resolve().parent/'runtime/world/source_manifest.json'
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2))
    print(json.dumps({'models': len(result['models']), 'collisions': len(result['collisions']),
                      'visuals': len(result['visuals']), 'markers': len(result['markers']),
                      'assets': len(result['assets']), 'output': str(output)}, indent=2))


if __name__ == '__main__':
    main()
