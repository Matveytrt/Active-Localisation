"""Read this repository's triangulated Z-up shelf without changing its geometry.

This is deliberately strict: unsupported transforms/topology fail, rather than
silently producing a different shelf. No external converter or team asset.
"""
import xml.etree.ElementTree as ET
import numpy as np


def shelf_meshes(path):
    root = ET.parse(path).getroot()
    for element in root.iter():
        element.tag = element.tag.split('}')[-1]
    assert root.findtext('asset/up_axis') == 'Z_UP'
    unit = root.find('asset/unit')
    assert unit is None or float(unit.get('meter', 1)) == 1
    for element in root.iter():
        if element.tag in ('translate', 'rotate', 'scale', 'lookat', 'skew'):
            raise ValueError('Unsupported Collada scene transform')
        if element.tag == 'matrix':
            assert np.allclose(np.fromstring(element.text, sep=' ').reshape(4, 4), np.eye(4))
    instances = root.findall('.//instance_geometry')
    geometries = root.findall('library_geometries/geometry')
    assert sorted(i.get('url')[1:] for i in instances) == sorted(g.get('id') for g in geometries)
    meshes = []
    for geometry in geometries:
        mesh = geometry.find('mesh')
        assert not mesh.findall('polylist') and not mesh.findall('polygons')
        for triangles in mesh.findall('triangles'):
            inputs = triangles.findall('input')
            assert len(inputs) == 1 and inputs[0].get('semantic') == 'VERTEX' and inputs[0].get('offset') == '0'
            vertices = mesh.find("vertices[@id='"+inputs[0].get('source')[1:]+"']")
            position = vertices.find("input[@semantic='POSITION']").get('source')[1:]
            source = mesh.find("source[@id='"+position+"']")
            accessor = source.find('technique_common/accessor')
            assert int(accessor.get('stride', 1)) == 3
            points = np.fromstring(source.findtext('float_array'), sep=' ').reshape(-1, 3)
            indices = np.fromstring(triangles.findtext('p'), sep=' ', dtype=np.int32).reshape(-1, 3)
            assert len(indices) == int(triangles.get('count')) and indices.min() >= 0 and indices.max() < len(points)
            material_id = triangles.get('material')
            material = root.find("library_materials/material[@id='"+material_id+"']")
            effect_id = material.find('instance_effect').get('url')[1:]
            effect = root.find("library_effects/effect[@id='"+effect_id+"']")
            color = list(map(float, effect.findtext('.//diffuse/color').split()))
            meshes.append({'points': points, 'indices': indices, 'color': color[:3]})
    assert meshes
    return meshes
