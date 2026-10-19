#!/usr/bin/env python3
"""Offline road and collision geometry for fixed reference worlds.
Saved state/link poses are WORLD poses. Apply Collada transforms, units and SDF
scales once, then project flight-band collision surfaces to convex XY footprints."""

import functools
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import LineString, MultiPoint, Point
from shapely.ops import unary_union

SPAWNS = [(-14., -3.), (-14., 0.), (-14., 3.),
          (-17., -3.), (-17., 0.), (-17., 3.)]
MAPS = ('base0', 'base3', 'base4', 'base5', 'base6', 'base15', 'base16')
DEFAULT_WORLD_DIR = Path(os.environ.get('SWARM_REFERENCE_WORLDS',
    str(Path(__file__).resolve().parents[2] / 'swarm_simulation/worlds/reference')))


def pose_matrix(text):
    x, y, z, r, p, a = map(float, (text or '0 0 0 0 0 0').split())
    cr, sr, cp, sp, ca, sa = (math.cos(r), math.sin(r), math.cos(p),
                             math.sin(p), math.cos(a), math.sin(a))
    mat = np.eye(4)
    mat[:3, :3] = [[ca*cp, ca*sp*sr-sa*cr, ca*sp*cr+sa*sr],
                  [sa*cp, sa*sp*sr+ca*cr, sa*sp*cr-ca*sr],
                  [-sp, cp*sr, cp*cr]]
    mat[:3, 3] = [x, y, z]
    return mat


def transform(points, matrix):
    return points @ matrix[:3, :3].T + matrix[:3, 3]


def resolve_mesh(uri):
    if uri.startswith('file://'):
        return Path(uri[7:])
    if not uri.startswith('model://'):
        raise ValueError('Unsupported mesh URI: ' + uri)
    roots = [Path(p) for p in os.environ.get('GAZEBO_MODEL_PATH', '').split(':') if p]
    roots += [Path(__file__).resolve().parents[3] / 'assets/models',
              Path(__file__).resolve().parents[2] / 'swarm_simulation/models',
              Path.home() / '.gazebo/models']
    for root in roots:
        path = root / uri[8:]
        if path.is_file():
            return path
    raise FileNotFoundError(uri)


@functools.lru_cache(maxsize=None)
def collada_triangles(path):
    root = ET.parse(path).getroot()
    # Strip XML namespaces for the remainder of this local parser.
    for e in root.iter():
        e.tag = e.tag.split('}')[-1]
    geometries = {}
    for geometry in root.findall('library_geometries/geometry'):
        mesh = geometry.find('mesh')
        if mesh is None:
            continue
        sources = {}
        for s in mesh.findall('source'):
            fa, accessor = s.find('float_array'), s.find('technique_common/accessor')
            if fa is not None and accessor is not None:
                stride = int(accessor.get('stride', '1'))
                sources['#' + s.get('id')] = np.fromstring(fa.text, sep=' ').reshape(-1, stride)
        vertices = {('#' + v.get('id')): v.find("input[@semantic='POSITION']").get('source')
                    for v in mesh.findall('vertices')}
        triangles = []
        for primitive in mesh:
            if primitive.tag not in ('triangles', 'polylist', 'polygons'):
                continue
            inputs = primitive.findall('input')
            vertex = next(i for i in inputs if i.get('semantic') == 'VERTEX')
            stride = max(int(i.get('offset', '0')) for i in inputs) + 1
            offset = int(vertex.get('offset', '0'))
            points = sources[vertices[vertex.get('source')]][:, :3]
            for indices in primitive.findall('p'):
                ids = np.fromstring(indices.text, sep=' ', dtype=int).reshape(-1, stride)[:, offset]
                counts = ([3] * (len(ids)//3) if primitive.tag == 'triangles' else
                          list(map(int, primitive.findtext('vcount').split()))
                          if primitive.tag == 'polylist' else [len(ids)])
                start = 0
                for count in counts:
                    for j in range(1, count-1):
                        triangles.append(points[ids[[start, start+j, start+j+1]]])
                    start += count
        geometries['#' + geometry.get('id')] = np.asarray(triangles)

    def visit(node, parent):
        mat = parent.copy()
        for child in node:
            if child.tag == 'matrix':
                mat = mat @ np.fromstring(child.text, sep=' ').reshape(4, 4)
            elif child.tag == 'translate':
                m = np.eye(4)
                m[:3, 3] = list(map(float, child.text.split()))
                mat = mat @ m
            elif child.tag == 'scale':
                mat = mat @ np.diag(list(map(float, child.text.split())) + [1.])
            elif child.tag == 'rotate':
                x, y, z, angle = map(float, child.text.split())
                axis = np.array([x, y, z]); axis /= np.linalg.norm(axis)
                x, y, z = axis
                skew = np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]])
                a = math.radians(angle); m = np.eye(4)
                m[:3, :3] = (math.cos(a)*np.eye(3) + (1-math.cos(a))*np.outer(axis, axis)
                             + math.sin(a)*skew)
                mat = mat @ m
        for instance in node.findall('instance_geometry'):
            tris = geometries[instance.get('url')]
            if tris.size:
                chunks.append(transform(tris.reshape(-1, 3), mat).reshape(-1, 3, 3))
        for child in node.findall('node'):
            visit(child, mat)
        for instance in node.findall('instance_node'):
            target = root.find("library_nodes/node[@id='%s']" % instance.get('url')[1:])
            if target is None:
                raise ValueError('Missing Collada instance_node: ' + instance.get('url'))
            visit(target, mat)

    chunks = []
    scene_id = root.find('scene/instance_visual_scene').get('url')[1:]
    scene = root.find("library_visual_scenes/visual_scene[@id='%s']" % scene_id)
    for node in scene.findall('node'):
        visit(node, np.eye(4))
    if not chunks:
        raise ValueError('No collision triangles: ' + str(path))
    tris = np.concatenate(chunks)
    unit = float(root.find('asset/unit').get('meter', '1'))
    tris *= unit
    up_axis = root.findtext('asset/up_axis', 'Z_UP')
    if up_axis != 'Z_UP':
        raise ValueError('Unsupported Collada axis: ' + up_axis)
    return tris


def slab_footprint(triangles, low, high):
    # Keep triangles spanning the band, then clip edges at both height planes.
    tris = triangles[(triangles[:, :, 2].min(axis=1) <= high) &
                     (triangles[:, :, 2].max(axis=1) >= low)]
    if not len(tris):
        return None
    vertices = tris.reshape(-1, 3)
    chunks = [vertices[(vertices[:, 2] >= low) & (vertices[:, 2] <= high), :2]]
    for a, b in ((0, 1), (1, 2), (2, 0)):
        start, end = tris[:, a], tris[:, b]
        for plane in (low, high):
            mask = ((start[:, 2] < plane) & (end[:, 2] > plane) |
                    (start[:, 2] > plane) & (end[:, 2] < plane))
            p, q = start[mask], end[mask]
            t = (plane-p[:, 2])/(q[:, 2]-p[:, 2])
            chunks.append((p + t[:, None]*(q-p))[:, :2])
    points = np.concatenate(chunks)
    return MultiPoint(points).convex_hull if len(points) else None


def load_world(path, low=3.85, high=5.15):
    world = ET.parse(path).getroot().find('world')
    roads = []
    for road in world.findall('road'):
        pts = [tuple(map(float, p.text.split()[:2])) for p in road.findall('point')]
        roads.append((road.get('name'), LineString(pts), float(road.findtext('width')) / 2))
    road_area = unary_union([line.buffer(half) for _, line, half in roads])
    state = world.find('state')
    states = {m.get('name'): m for m in state.findall('model')} if state is not None else {}
    obstacles = []
    for model in world.findall('model'):
        name = model.get('name')
        if name in ('ground_plane',) or name.startswith(('city_terrain', 'ocean', 'typhoon_h480')):
            continue
        saved = states.get(name)
        saved_links = {l.get('name'): l for l in saved.findall('link')} if saved is not None else {}
        model_pose = pose_matrix(saved.findtext('pose') if saved is not None else model.findtext('pose'))
        for link in model.findall('link'):
            sl = saved_links.get(link.get('name'))
            link_pose = (pose_matrix(sl.findtext('pose')) if sl is not None else
                         model_pose @ pose_matrix(link.findtext('pose')))
            for collision in link.findall('collision'):
                geom = collision.find('geometry')
                mat = link_pose @ pose_matrix(collision.findtext('pose'))
                mesh = geom.find('mesh')
                if mesh is not None:
                    scale = np.array(list(map(float, mesh.findtext('scale', '1 1 1').split())))
                    tris = collada_triangles(resolve_mesh(mesh.findtext('uri'))) * scale
                    tris = transform(tris.reshape(-1, 3), mat).reshape(-1, 3, 3)
                    footprint = slab_footprint(tris, low, high)
                elif geom.find('polyline') is not None:
                    poly = geom.find('polyline')
                    # Road curbs in these worlds are only 0.15m high.
                    z = mat[2, 3]; height = float(poly.findtext('height', '1'))
                    if z + height < low or z > high:
                        continue
                    raise ValueError('Tall polyline collision: ' + name)
                elif geom.find('box') is not None:
                    size = np.array(list(map(float, geom.findtext('box/size').split())))
                    pts = transform(np.array([[x, y, z] for x in (-.5, .5)
                                              for y in (-.5, .5) for z in (-.5, .5)])*size, mat)
                    if pts[:, 2].max() < low or pts[:, 2].min() > high:
                        continue
                    footprint = MultiPoint(pts[:, :2]).convex_hull
                elif geom.find('cylinder') is not None or geom.find('sphere') is not None:
                    kind = 'cylinder' if geom.find('cylinder') is not None else 'sphere'
                    radius = float(geom.findtext(kind + '/radius'))
                    height = float(geom.findtext(kind + '/length')) if kind == 'cylinder' else 2*radius
                    if mat[2, 3]+height/2 < low or mat[2, 3]-height/2 > high:
                        continue
                    footprint = Point(mat[:2, 3]).buffer(radius)
                else:
                    raise ValueError('Unsupported collision geometry: ' + name)
                if footprint is not None and not footprint.is_empty:
                    obstacles.append((name, footprint))
    return roads, road_area, obstacles
