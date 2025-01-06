from shapely.geometry import Polygon, MultiLineString
from shapely import affinity
import numpy as np


def optimal_sweep_angle(poly, coords, show_plot=False):
    """Finds optimal sweep angle to reduce the amount of total turns in the sweep path. This reduces energy wasted."""
    og_miny_idx = coords[:,1].argmin()
    origin_vtx, origin_idx = get_lowest_pair(coords)
    # print(origin_vtx)
    poly = affinity.translate(poly, -origin_vtx[0], -origin_vtx[1])
    origin_vtx = np.asfarray([0.0,0.0])
    og_vtx = origin_vtx
    rotated_poly = poly
    dang_max = np.pi/2000
    curr_angle = 0.0
    opt_angle = 0.0
    min_d = 999999999999.

    if show_plot:
        import matplotlib.pyplot as plt
        plt.cla()
        plt_cnt = 0
        dls = []
        dang_ls = []
        fig, ax = plt.subplots(1,1)
        ax.set_aspect('equal')
        plt.draw()
        ax.plot(*poly.exterior.xy)
    while True:
        rot_coords = np.asfarray(rotated_poly.exterior.coords)
        filt_coords = np.delete(rot_coords, origin_idx, axis=0)
        filt_coords = filt_coords[filt_coords[:,0]>origin_vtx[0],:]  # only take vertices to the right since the polygon is rolled right
        dang = dang_max
        if len(filt_coords):  # skip if origin is the furthest right
            miny_idx = filt_coords[:,1].argmin()
            miny = filt_coords[miny_idx,1]
            if np.isclose(miny, origin_vtx[1], atol=0.001):
                origin_vtx = filt_coords[miny_idx,:]
                origin_idx = miny_idx
            if miny <= 0.5 and miny > 0.0:
                dang = np.pi/100000
        rotated_poly = affinity.rotate(rotated_poly, -dang, origin=origin_vtx, use_radians=True)
        d = max(rotated_poly.exterior.coords.xy[1])
        curr_angle += dang
        if d < min_d:
            min_d = d
            opt_angle = curr_angle
        shift = og_vtx - np.asfarray(rotated_poly.exterior.coords)[og_miny_idx,:]
        translated_poly = affinity.translate(rotated_poly, shift[0], shift[1])
        if poly.equals_exact(translated_poly, 0.5 * 10 ** (-1)):
            break
        if show_plot and plt_cnt % 10 == 0:
            plt.cla()
            ax.plot(*rotated_poly.exterior.xy)
            plt.pause(0.0001)
            plt.draw()
        if show_plot:
            dls.append(d)
            if dang_ls:
                dang_ls.append(dang+dang_ls[-1])
            else:
                dang_ls.append(dang)
            plt_cnt = plt_cnt + 1

    if show_plot:
        plt.cla()
        ax = plt.gca()
        ax.plot(dang_ls,dls)
        plt.show()
    return opt_angle, min_d


def get_lowest_pair(pairs):
    """Finds the lowest and furthest left x,y pair."""
    miny = pairs[:,1].argmin()
    coords_miny = pairs[miny,:]
    if coords_miny.ndim == 1:
        lowest_vtx = coords_miny
        lowest_idx = [miny, ]
    else:
        lowest_vtx = coords_miny[coords_miny[:,0].argmin(),:]
    return lowest_vtx, lowest_idx


def sweep_polygon(poly: Polygon, separation:float=2.0, start_left=False):
    """Generates path that sweeps across polygon.

    Derived from paper https://grvc.us.es/comets/papers/MAZA-DARS-2004.pdf and "Multiple UAV area decomposition and coverage" JF Araujo.

    Generic paper with coverage path planning ideas https://www.mdpi.com/2504-446X/3/1/4/htm#B34-drones-03-00004.

    Args:
        poly (Polygon): Polygon to sweep across.
        separation (float, optional): Separation between points of interest. Defines spread of parallel paths and the individual points along those paths. Defaults to 2.0.
    """
    alt = 1.0
    cam_ang = -np.pi/2
    fovh = np.pi/2
    fovw = np.pi/2
    # sw_i = 2*alt*np.tan(fovw)*(np.sin(cam_ang)+np.cos(cam_ang)*np.tan(np.pi/2-cam_ang-fovh))  #eq.1, sensing width
    sw_i = separation
    coords = np.asfarray(poly.exterior.coords)
    opt_angle, min_d = optimal_sweep_angle(poly, coords, show_plot=False)
    rot_poly = affinity.rotate(poly, -opt_angle, origin='center', use_radians=True)  #eq.23
    nl_i = int(np.ceil(min_d/sw_i))  #eq.25  number of lanes
    os_i = (min_d - nl_i * sw_i)/2  #eq.26 offset of first lane
    minx, miny, maxx, maxy = rot_poly.bounds
    stops = int(np.floor((maxx - minx)/sw_i))
    lane_y = np.tile(np.arange(0, nl_i)*sw_i + os_i + miny, (2, 1)).T
    lane_x = np.tile(np.linspace(minx, maxx, num=2), (nl_i, 1))
    np_lines = np.dstack((lane_x.reshape(nl_i,2,1),lane_y.reshape(nl_i,2,1)))
    sp_lines = MultiLineString(list(np_lines))
    # TODO: rescale polygon to include a border of 0.5 sw_i
    scale_poly = affinity.scale(rot_poly, xfact=0.9, yfact=0.9)
    sp_lines = sp_lines.intersection(scale_poly)
    og_lines = affinity.rotate(sp_lines, opt_angle, origin=get_poly_center(poly), use_radians=True)
    flipped = start_left
    path = np.empty((0,2))
    for l in og_lines.geoms:
        l_np = np.asfarray(l)
        if flipped: l_np = np.flip(l_np, axis=0)
        l_dist = np.linalg.norm(l_np[1,:]-l_np[0,:])
        stops = int(np.ceil(l_dist/sw_i))
        l_path = np.linspace(l_np[0,:],l_np[1,:],num=stops)
        path = np.append(path, l_path, axis=0)
        flipped = not flipped
    return path


def get_poly_center(poly:Polygon):
    minx, miny, maxx, maxy = poly.bounds
    return (minx+(maxx-minx)/2,miny+(maxy-miny)/2)


def plot_path_on_poly(poly, path, ax=None, show=True):
    import matplotlib.pyplot as plt
    if ax is None: ax = plt.gca()
    ax.plot(path[:,0],path[:,1])
    ax.plot(*poly.exterior.xy)
    if show: plt.show()


def main():
    import matplotlib.pyplot as plt
    from robot_command.split_polygon import split_polygon_voronoi, plot_geom_collection
    # poly = Polygon([[10, 10], [40, 0], [60,20], [20, 50], [0, 30], [10, 10]])
    poly = Polygon([[0,0], [30,-10], [23,27], [0,10], [0,0]])
    regions = split_polygon_voronoi(poly, 3)
    for r in regions.geoms:
        path = sweep_polygon(r)
        plot_path_on_poly(r, path, show=False)
    plt.show()
    # plot_geom_collection(regions)


if __name__=="__main__":
    main()
