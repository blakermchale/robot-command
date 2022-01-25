import shapely
import numpy as np
from shapely.geometry import Polygon, MultiPoint, GeometryCollection
from scipy.cluster.vq import kmeans, whiten
from shapely.ops import voronoi_diagram
from enum import IntEnum, auto


class SplitMethod(IntEnum):
    VORONOI=0
    DELAUNAY=auto()


def generate_random_pts(polygon: Polygon, total):
    points = np.empty((0,2))
    tris = GeometryCollection(shapely.ops.triangulate(polygon))
    total_area = polygon.area
    for t in tris.geoms:
        v = np.asfarray(t.exterior.coords.xy).T[:3]  # get triangle vertices
        n_pts = int(np.ceil((t.area/total_area)*total))
        p = points_on_triangle(v, n_pts)
        points = np.append(points, p, axis=0)
    
    # minx, miny, maxx, maxy = polygon.bounds
    # uniform_pts = np.random.uniform([minx, miny], [maxx, maxy], size=(total,2))
    # u_mp = MultiPoint(uniform_pts)
    # points = u_mp.intersection(polygon)
    # uniform_pts = np.linspace([minx, miny], [maxx, maxy], num=total)
    # while len(points) < total:
    #     pnt = Point(random.uniform(minx, maxx), random.uniform(miny, maxy))
    #     # pnt = np.array([random.uniform(minx, maxx), random.uniform(miny, maxy)])
    #     if polygon.contains(pnt):
    #         points.append([pnt.x, pnt.y])
    return np.asfarray(points)


def generate_centroids(points: np.ndarray, num_centroids):
    whitened_pts = whiten(points)
    codebook, distortion = kmeans(whitened_pts, num_centroids)
    # plt.scatter(whitened_pts[:, 0], whitened_pts[:, 1])
    # plt.scatter(codebook[:, 0], codebook[:, 1], c='r')
    # plt.show()
    return MultiPoint(codebook * np.std(points, 0))


def points_on_triangle(v, n):
    """
    Give n random points uniformly on a triangle. From https://stackoverflow.com/a/47418580.

    The vertices of the triangle are given by the shape
    (2, 3) array *v*: one vertex per row.
    """
    x = np.sort(np.random.rand(2, n), axis=0)
    return np.column_stack([x[0], x[1]-x[0], 1.0-x[1]]) @ v


def split_polygon_voronoi(poly:Polygon, splits:int, samples:int=100000):
    rnd_pts = generate_random_pts(poly, samples)
    centroids = generate_centroids(rnd_pts, splits)
    regions = voronoi_diagram(centroids)
    regions = GeometryCollection([poly.intersection(region) for region in regions.geoms])
    return regions


def plot_geom_collection(regions: GeometryCollection, ax=None, show=True):
    import matplotlib.pyplot as plt
    if ax is None:
        ax = plt.gca()
    for region in regions.geoms:
        ax.plot(*region.exterior.xy)
    if show:
        plt.show()


if __name__=="__main__":
    # poly = Polygon([[0, 0], [30, 0], [30, 20], [0, 20], [0, 0]])
    # poly = Polygon([[0, 0], [40, 0], [20, 20], [0, 30], [0, 0]])
    poly = Polygon([[10, 10], [40, 0], [60,20], [20, 50], [0, 30], [10, 10]])
    regions = split_polygon_voronoi(poly, 5)
    plot_geom_collection(regions)
