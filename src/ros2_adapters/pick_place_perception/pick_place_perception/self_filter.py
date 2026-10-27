"""Self masks from supplied URDF collisions, never visual or truth geometry.

STL convex hulls are only a broad-phase test; ray parity tests actual triangles.
Bodies remain separate, especially gripper fingers. Unclosed meshes never carve
unknown cells, since their interiors cannot be established reliably.
"""

from dataclasses import dataclass, replace
import hashlib
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
import yaml

from .obstacle_geometry import transform


@dataclass
class Body:
    link: str
    origin: np.ndarray
    kind: str
    dimensions: np.ndarray
    equations: object = None
    inverse: object = None
    ray_data: object = None
    closed: bool = True
    triangles: object = None
    surface_tree: object = None
    triangle_radius: float = 0.
    surface_groups: object = None
    surface_bounds: object = None

    def at(self, link_pose):
        return replace(self, inverse=np.linalg.inv(link_pose @ self.origin))

    def surface_contains(self, points, tolerance):
        """Point-only mask witnessed by distance to an actual supplied triangle.

        Never use this shell to clear unknown voxels or extend visibility rays.
        The broad-phase radius includes every triangle that could be within
        tolerance, including long triangles whose centroids are far away.
        """
        if tolerance <= 0:
            return self.contains(points)
        # The acquired robot returns usually lie on a surface. Check those
        # first so parity rays are needed only for the remaining points. This
        # is the same union, without rechecking already witnessed surfaces.
        result = self.matches_surface(points, tolerance)
        remaining = np.flatnonzero(~result)
        if len(remaining):
            result[remaining] = self.contains(points[remaining])
        return result

    def matches_surface(self, points, tolerance):
        """Finite return near a real surface, never an inferred mesh interior."""
        p = transform(points, self.inverse)
        if self.kind == 'box':
            delta = np.abs(p)-self.dimensions/2
            signed = np.linalg.norm(np.maximum(delta, 0), axis=1)+np.minimum(delta.max(axis=1), 0)
            return np.abs(signed) <= tolerance
        if self.kind == 'sphere':
            return np.abs(np.linalg.norm(p, axis=1)-self.dimensions[0]) <= tolerance
        if self.kind == 'cylinder':
            delta = np.column_stack((np.linalg.norm(p[:, :2], axis=1)-self.dimensions[0],
                                     np.abs(p[:, 2])-self.dimensions[1]/2))
            signed = np.linalg.norm(np.maximum(delta, 0), axis=1)+np.minimum(delta.max(axis=1), 0)
            return np.abs(signed) <= tolerance
        result = np.zeros(len(points), dtype=bool)
        if self.surface_tree is None and self.surface_groups is None:
            return result
        indices = np.flatnonzero(~result & ((p >= self.dimensions[0]-tolerance)
                                           & (p <= self.dimensions[1]+tolerance)).all(axis=1))
        if not len(indices):
            return result
        triangle_lower, triangle_upper = (self.surface_bounds if self.surface_bounds is not None else
                                          (self.triangles.min(axis=1), self.triangles.max(axis=1)))
        groups = self.surface_groups
        if groups is None:
            groups = [(self.surface_tree, np.arange(len(self.triangles)), self.triangle_radius)]
        # Bounded batches keep the full candidate set while avoiding a Python
        # triangle scan for each acquired pixel. No nearest-k truncation.
        for tree, triangle_ids, radius in groups:
            remaining = indices[~result[indices]]
            for start in range(0, len(remaining), 256):
                selected = remaining[start:start+256]
                candidates = tree.query_ball_point(p[selected], radius+tolerance+1e-12)
                counts = np.array([len(choices) for choices in candidates])
                if not counts.sum():
                    continue
                point_indices = np.repeat(selected, counts)
                triangle_indices = triangle_ids[np.concatenate(candidates).astype(int)]
                near = ((p[point_indices] >= triangle_lower[triangle_indices]-tolerance)
                        & (p[point_indices] <= triangle_upper[triangle_indices]+tolerance)).all(axis=1)
                point_indices, triangle_indices = point_indices[near], triangle_indices[near]
                if len(point_indices):
                    close = triangle_distance_squared(p[point_indices], self.triangles[triangle_indices]) <= tolerance*tolerance
                    result[point_indices[close]] = True
        return result

    def index_surfaces(self, triangles):
        """Complete radius buckets prevent one long face widening every query."""
        self.triangles = triangles
        self.surface_bounds = (triangles.min(axis=1), triangles.max(axis=1))
        centers = triangles.mean(axis=1)
        radii = np.linalg.norm(triangles-centers[:, None], axis=2).max(axis=1)
        buckets = np.frexp(radii)[1]
        self.surface_groups = []
        for bucket in np.unique(buckets):
            ids = np.flatnonzero(buckets == bucket)
            self.surface_groups.append((cKDTree(centers[ids]), ids, float(radii[ids].max())))

    def contains(self, points):
        p = transform(points, self.inverse)
        if self.kind == 'box':
            return (np.abs(p) <= self.dimensions/2 + 1e-7).all(axis=1)
        if self.kind == 'sphere':
            return np.linalg.norm(p, axis=1) <= self.dimensions[0]+1e-7
        if self.kind == 'cylinder':
            return (np.linalg.norm(p[:, :2], axis=1) <= self.dimensions[0]+1e-7) & (np.abs(p[:, 2]) <= self.dimensions[1]/2+1e-7)
        inside = ((p >= self.dimensions[0]-1e-7) & (p <= self.dimensions[1]+1e-7)).all(axis=1)
        indices = np.flatnonzero(inside)
        if len(indices):
            # einsum avoids large BLAS thread pools for small link clouds.
            inside[indices] = (np.einsum('ij,kj->ik', p[indices], self.equations[:, :3])
                               + self.equations[:, 3] <= 1e-7).all(axis=1)
        if self.ray_data is not None:
            basis, a, ab, ac, determinant, lo, hi = self.ray_data
            for i in np.flatnonzero(inside):
                point = p[i] @ basis
                candidates = ((point[:2] >= lo-1e-10) & (point[:2] <= hi+1e-10)).all(axis=1)
                delta = point[:2]-a[candidates, :2]
                b, c, d = ab[candidates], ac[candidates], determinant[candidates]
                u = (delta[:, 0]*c[:, 1]-delta[:, 1]*c[:, 0])/d
                v = (b[:, 0]*delta[:, 1]-b[:, 1]*delta[:, 0])/d
                hits = (u >= -1e-10) & (v >= -1e-10) & (u+v <= 1+1e-10)
                heights = a[candidates, 2]+u*b[:, 2]+v*c[:, 2]
                heights = heights[hits]
                # Shared triangle edges can generate two identical crossings.
                crossings = np.unique(np.round(heights[heights > point[2]+1e-7], 9))
                inside[i] = (np.any(np.abs(heights-point[2]) <= 1e-7) or len(crossings) % 2 == 1)
        return inside


def triangle_distance_squared(point, triangles):
    """Exact distance to triangle faces and their bounded edges; no hull fill."""
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    e, f, delta = b-a, c-a, point-a
    normal = np.cross(e, f)
    normal2 = np.einsum('ij,ij->i', normal, normal)
    distance = np.full(len(triangles), np.inf)
    valid = normal2 > 1e-24
    if valid.any():
        n, d, ee, ff = normal[valid], delta[valid], e[valid], f[valid]
        height = np.einsum('ij,ij->i', d, n)/normal2[valid]
        projected = d-height[:, None]*n
        e2 = np.einsum('ij,ij->i', ee, ee)
        ef = np.einsum('ij,ij->i', ee, ff)
        f2 = np.einsum('ij,ij->i', ff, ff)
        pe = np.einsum('ij,ij->i', projected, ee)
        pf = np.einsum('ij,ij->i', projected, ff)
        denominator = e2*f2-ef*ef
        u, v = (pe*f2-pf*ef)/denominator, (pf*e2-pe*ef)/denominator
        distance[valid] = np.where((u >= 0) & (v >= 0) & (u+v <= 1),
                                   height*height*normal2[valid], np.inf)
    for start, end in ((a, b), (a, c), (b, c)):
        edge, delta = end-start, point-start
        length2 = np.einsum('ij,ij->i', edge, edge)
        along = np.divide(np.einsum('ij,ij->i', delta, edge), length2,
                          out=np.zeros(len(edge)), where=length2 > 0)
        residual = delta-np.clip(along, 0, 1)[:, None]*edge
        distance = np.minimum(distance, np.einsum('ij,ij->i', residual, residual))
    return distance


def origin(xyz='0 0 0', rpy='0 0 0'):
    xyz, rpy = np.asarray(str(xyz).split(), float), np.asarray(str(rpy).split(), float)
    if xyz.shape != (3,) or rpy.shape != (3,) or not np.isfinite([xyz, rpy]).all():
        raise ValueError('invalid self-filter origin')
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def stl_vertices(path):
    data = Path(path).read_bytes()
    count = struct.unpack_from('<I', data, 80)[0] if len(data) >= 84 else 0
    if count and len(data) == 84+50*count:
        dtype = np.dtype([('normal', '<f4', (3,)), ('vertices', '<f4', (3, 3)), ('attr', '<u2')])
        vertices = np.frombuffer(data, dtype=dtype, offset=84)['vertices'].reshape(-1, 3).astype(float)
    else:
        vertices = np.array([list(map(float, line.split()[1:]))
                             for line in data.decode('ascii').splitlines()
                             if line.strip().startswith('vertex ')])
    if vertices.ndim != 2 or vertices.shape[1] != 3 or len(vertices) < 4 or not np.isfinite(vertices).all():
        raise ValueError(f'invalid collision STL: {path}')
    return np.unique(vertices, axis=0), data, vertices.reshape(-1, 3, 3)


def prepare_rays(triangles):
    # Deterministic non-axis ray reduces degeneracy on CAD-aligned faces.
    direction = np.array([.371, .529, .763])
    direction /= np.linalg.norm(direction)
    x = np.cross(direction, [0, 0, 1.])
    x /= np.linalg.norm(x)
    basis = np.column_stack((x, np.cross(direction, x), direction))
    _, indices = np.unique(triangles.reshape(-1, 3), axis=0, return_inverse=True)
    faces = indices.reshape(-1, 3)
    nondegenerate = np.linalg.norm(np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0]), axis=1) > 1e-12
    faces, triangles = faces[nondegenerate], triangles[nondegenerate]
    edges = np.sort(np.concatenate((faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]])), axis=1)
    _, counts = np.unique(edges, axis=0, return_counts=True)
    closed = bool(len(counts) and np.all(counts == 2))
    t = triangles @ basis
    a, ab, ac = t[:, 0], t[:, 1]-t[:, 0], t[:, 2]-t[:, 0]
    determinant = ab[:, 0]*ac[:, 1]-ab[:, 1]*ac[:, 0]
    keep = np.abs(determinant) > 1e-14
    return (basis, a[keep], ab[keep], ac[keep], determinant[keep],
            t[keep, :, :2].min(axis=1), t[keep, :, :2].max(axis=1)), closed


def mesh_path(uri, root):
    if uri.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package, rest = uri[len('package://'):].split('/', 1)
        return Path(get_package_share_directory(package)) / rest
    if uri.startswith('file://'):
        return Path(uri[len('file://'):])
    path = Path(uri)
    return path if path.is_absolute() else root/path


def load_model(path):
    """YAML selects explicit URDF and required links; supplemental collision boxes
    must be declared for sensor bodies that have no URDF collision element.
    """
    path = Path(path)
    config = yaml.safe_load(path.read_text())
    if (not isinstance(config, dict) or set(config) != {'robot_urdf', 'required_links', 'extra_boxes'}
            or not isinstance(config['required_links'], list) or not config['required_links']):
        raise ValueError('self-filter model requires robot_urdf, required_links and extra_boxes')
    robot_file = Path(config['robot_urdf'])
    if not robot_file.is_absolute():
        robot_file = path.parent/robot_file
    data = robot_file.read_bytes()
    robot = ET.fromstring(data)
    digest = hashlib.sha256(data+path.read_bytes())
    links = {link.attrib['name'] for link in robot.findall('link')}
    joints = {j.attrib['name'] for j in robot.findall('joint')
              if j.attrib['type'] in ('continuous', 'revolute', 'prismatic') and j.find('mimic') is None}
    bodies = []
    for link in robot.findall('link'):
        for collision in link.findall('collision'):
            pose = collision.find('origin')
            pose = origin(**pose.attrib) if pose is not None else np.eye(4)
            geometry = collision.find('geometry')
            if geometry is None or len(geometry) != 1:
                raise ValueError('unsupported collision geometry')
            shape = geometry[0]
            kind, equations, ray_data, closed = shape.tag, None, None, True
            if kind == 'mesh':
                filename = mesh_path(shape.attrib['filename'], robot_file.parent)
                if filename.suffix.lower() != '.stl':
                    raise ValueError('self filter supports collision meshes in STL only')
                vertices, raw, triangles = stl_vertices(filename)
                digest.update(raw)
                scale = np.array(shape.attrib.get('scale', '1 1 1').split(), float)
                if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
                    raise ValueError('invalid mesh scale')
                vertices *= scale
                ray_data, closed = prepare_rays(triangles*scale)
                equations = ConvexHull(vertices).equations
                dimensions = np.array([vertices.min(axis=0), vertices.max(axis=0)])
            elif kind == 'box':
                dimensions = np.array(shape.attrib['size'].split(), float)
            elif kind == 'sphere':
                dimensions = np.array([float(shape.attrib['radius'])])
            elif kind == 'cylinder':
                dimensions = np.array([float(shape.attrib['radius']), float(shape.attrib['length'])])
            else:
                raise ValueError(f'unsupported self collision: {kind}')
            if not np.isfinite(dimensions).all() or (kind != 'mesh' and np.any(dimensions <= 0)):
                raise ValueError('invalid collision dimensions')
            if kind == 'box' and dimensions.shape != (3,):
                raise ValueError('invalid box dimensions')
            bodies.append(Body(link.attrib['name'], pose, kind, dimensions, equations, ray_data=ray_data, closed=closed))
            if kind == 'mesh':
                bodies[-1].index_surfaces(triangles*scale)
    for extra in config['extra_boxes']:
        if set(extra) != {'link', 'size', 'xyz', 'rpy'} or extra['link'] not in links:
            raise ValueError('supplemental collision must name a URDF link')
        size = np.array(extra['size'], float)
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError('invalid supplemental collision box')
        pose = origin(' '.join(map(str, extra['xyz'])), ' '.join(map(str, extra['rpy'])))
        bodies.append(Body(extra['link'], pose, 'box', size))
    if not set(config['required_links']).issubset({body.link for body in bodies}):
        raise ValueError('required robot/tool/camera collision body missing')
    return bodies, joints, digest.hexdigest()
