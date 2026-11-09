"""Author the project's own SDF visual/collision scene into a standalone USD."""
import json
from pathlib import Path
import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
from world_source import audit
from collada_source import shelf_meshes


def set_matrix(prim, matrix):
    UsdGeom.Xformable(prim).AddTransformOp().Set(Gf.Matrix4d(*np.asarray(matrix).T.flatten().tolist()))


def material(stage, path, color=(.6, .6, .6), texture=None):
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(.9)
    if texture is None:
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    else:
        reader = UsdShade.Shader.Define(stage, path+'/UV')
        reader.CreateIdAttr('UsdPrimvarReader_float2')
        reader.CreateInput('varname', Sdf.ValueTypeNames.Token).Set('st')
        tex = UsdShade.Shader.Define(stage, path+'/Texture')
        tex.CreateIdAttr('UsdUVTexture')
        tex.CreateInput('file', Sdf.ValueTypeNames.Asset).Set(Sdf.AssetPath(str(texture)))
        tex.CreateInput('sourceColorSpace', Sdf.ValueTypeNames.Token).Set('sRGB')
        tex.CreateInput('st', Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), 'result')
        # Original OGRE marker materials explicitly specify "lighting off".
        shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0., 0., 0.))
        shader.CreateInput('emissiveColor', Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), 'rgb')
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def cube(stage, path, transform, size, mat=None, collision=False):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1.)
    matrix = np.array(transform).copy()
    matrix[:3, :3] = matrix[:3, :3] @ np.diag(size)
    set_matrix(shape.GetPrim(), matrix)
    if mat:
        UsdShade.MaterialBindingAPI.Apply(shape.GetPrim()).Bind(mat)
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    return shape


def build(repo):
    repo = Path(repo)
    output = repo/'isaac/runtime/world'
    output.mkdir(parents=True, exist_ok=True)
    manifest = audit(repo)
    path = output/'own_world.usdc'
    stage = Usd.Stage.CreateNew(str(path))
    root = UsdGeom.Xform.Define(stage, '/Environment')
    stage.SetDefaultPrim(root.GetPrim())
    UsdGeom.SetStageMetersPerUnit(stage, 1.)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    gray = material(stage, '/Environment/Materials/Gray')
    white = material(stage, '/Environment/Materials/White', (1., 1., 1.))
    green = material(stage, '/Environment/Materials/Green', (.1, .8, .1))
    for i, entry in enumerate(manifest['collisions']):
        transform = np.array(entry['transform'])
        if entry['type'] == 'plane':
            assert entry['normal'] == [0., 0., 1.]
            transform[2, 3] -= .1
            size = [*entry['size'], .2]
        else:
            assert entry['type'] == 'box'
            size = entry['size']
        shape = cube(stage, '/Environment/Collisions/C'+str(i), transform, size, collision=True)
        # Physics continues to query these prims; rendering uses original visuals.
        shape.CreateVisibilityAttr('invisible')
        shape.GetPrim().CreateAttribute('sourceModel', Sdf.ValueTypeNames.String).Set(entry['model'])
    marker_by_model = {m['model']: m for m in manifest['markers']}
    mesh_cache = {}
    for i, entry in enumerate(manifest['visuals']):
        base = '/Environment/Visuals/V'+str(i)
        transform = np.array(entry['transform'])
        if entry['type'] == 'plane':
            transform[2, 3] -= .1
            cube(stage, base, transform, [*entry['size'], .2], gray)
        elif entry['type'] == 'box':
            cube(stage, base, transform, entry['size'], green if entry['model'] == 'spawn_area_marker' else white)
            if entry['model'] in marker_by_model:
                marker = marker_by_model[entry['model']]
                mat = material(stage, '/Environment/Materials/'+entry['model'], texture=repo/marker['texture'])
                parent = UsdGeom.Xform.Define(stage, base+'Face')
                set_matrix(parent.GetPrim(), transform)
                sx, sy, sz = entry['size']
                face = UsdGeom.Mesh.Define(stage, base+'Face/Image')
                face.CreatePointsAttr([(-sx/2, -sy/2, sz/2+.00001), (sx/2, -sy/2, sz/2+.00001),
                                       (sx/2, sy/2, sz/2+.00001), (-sx/2, sy/2, sz/2+.00001)])
                face.CreateFaceVertexCountsAttr([4])
                face.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
                face.CreateSubdivisionSchemeAttr('none')
                face.CreateDoubleSidedAttr(False)
                UsdGeom.PrimvarsAPI(face).CreatePrimvar('st', Sdf.ValueTypeNames.TexCoord2fArray,
                    UsdGeom.Tokens.vertex).Set([(0, 0), (1, 0), (1, 1), (0, 1)])
                UsdShade.MaterialBindingAPI.Apply(face.GetPrim()).Bind(mat)
        elif entry['type'] == 'mesh':
            if entry['path'] not in mesh_cache:
                mesh_cache[entry['path']] = shelf_meshes(repo/entry['path'])
            matrix = transform.copy()
            matrix[:3, :3] = matrix[:3, :3] @ np.diag(entry['scale'])
            parent = UsdGeom.Xform.Define(stage, base)
            set_matrix(parent.GetPrim(), matrix)
            for j, source in enumerate(mesh_cache[entry['path']]):
                mesh = UsdGeom.Mesh.Define(stage, base+'/Part'+str(j))
                mesh.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(source['points'].astype(np.float32)))
                mesh.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(np.full(len(source['indices']), 3, dtype=np.int32)))
                mesh.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(source['indices'].ravel()))
                mesh.CreateSubdivisionSchemeAttr('none')
                mat = material(stage, '/Environment/Materials/Shelf'+str(j), source['color'])
                UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
        else:
            raise ValueError(entry['type'])
    errors = []
    cache = UsdGeom.XformCache()
    for i, entry in enumerate(manifest['collisions']):
        expected = np.array(entry['transform'])
        size = entry['size']
        if entry['type'] == 'plane':
            expected[2, 3] -= .1
            size = [*size, .2]
        expected[:3, :3] = expected[:3, :3] @ np.diag(size)
        prim = stage.GetPrimAtPath('/Environment/Collisions/C'+str(i))
        assert prim.HasAPI(UsdPhysics.CollisionAPI)
        errors.append(float(np.max(abs(np.asarray(cache.GetLocalToWorldTransform(prim)).T-expected))))
    marker_faces = [p for p in stage.Traverse() if p.GetName() == 'Image' and p.IsA(UsdGeom.Mesh)]
    shelf_parts = [p for p in stage.Traverse() if p.GetName().startswith('Part') and p.IsA(UsdGeom.Mesh)]
    assert len(marker_faces) == 24 and len(shelf_parts) == 12 and max(errors) < 1e-6
    manifest['usd_validation'] = {'collisions': len(errors), 'max_transform_error': max(errors),
        'marker_faces': len(marker_faces), 'shelf_mesh_parts': len(shelf_parts),
        'shelf_triangles_total': sum(len(UsdGeom.Mesh(p).GetFaceVertexCountsAttr().Get()) for p in shelf_parts)}
    stage.GetRootLayer().Save()
    manifest['usd'] = str(path)
    manifest['mesh_triangles_per_shelf'] = sum(len(m['indices']) for group in mesh_cache.values() for m in group)
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2))
    return path, manifest
