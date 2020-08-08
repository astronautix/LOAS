import memoization
import copy
import numpy as np
import trimesh
import shapely.ops
import math
import loas

@memoization.cached
def projected_area(mesh, normal):
    normal = normal/np.linalg.norm(normal)
    m = copy.deepcopy(mesh)
    dir_rot = normal + loas.Vec(1,0,0)
    if np.linalg.norm(dir_rot) < 1e-6:
        dir_rot = loas.Vec(0,1,0)
    m.apply_transform(trimesh.transformations.rotation_matrix(math.pi, dir_rot.line()))
    m.apply_transform(trimesh.transformations.projection_matrix((0,0,0),(1,0,0)))
    polygons = [
        shapely.geometry.Polygon(triangle[:,1:])
        for index_triangle, triangle in enumerate(m.triangles)
        if np.linalg.norm(m.face_normals[index_triangle] - np.array([1,0,0])) < 1e-6
    ]
    poly_merged = shapely.ops.unary_union(polygons)
    return poly_merged.area
