from math import pi, sin, cos, sqrt, atan2, asin
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import cm, colors



def to_deg(rad):
    return rad * 360 / 2 / pi

def to_rad(deg):
    return deg / 360 * 2 * pi

def rotate(vec, lat, ha, de):
    # vec is defined in the mount frame (+Xm=pointing direction, +Ym=dec axis toward the east when pointing south, +Zm=X^Y)
    # compute its coordinates in the dome frame (east, north, zenith)

    # coordinate in interim frame (+Xe=equator,+Ym,+Ze=Xe^Zm)
    de_vec = np.array([cos(de) * vec[0] - sin(de) * vec[2], vec[1], cos(de) * vec[2] + sin(de) * vec[0]])

    # coordinate in interim frame (+Xs=equator south,+Ys=Ze^Xs,+Ze)
    ra_vec = np.array([cos(ha) * de_vec[0] + sin(ha) * de_vec[1], cos(ha) * de_vec[1] - sin(ha) * de_vec[0], de_vec[2]])

    #ref_vec = np.array([cos(lat) * ra_vec[2] - sin(lat) * ra_vec[0], ra_vec[1], cos(lat) * ra_vec[0] + sin(lat) * ra_vec[2]])
    ref_vec = np.array([ra_vec[1], cos(lat) * ra_vec[2] - sin(lat) * ra_vec[0], cos(lat) * ra_vec[0] + sin(lat) * ra_vec[2]])
    return ref_vec

def reverse(az,el,lat):
    # compute ha and de from az and el
    vec = np.array([cos(el)*sin(az), cos(el)*cos(az), sin(el)])

    ra_vec = np.array([-vec[1]*sin(lat)+vec[2]*cos(lat), vec[0], vec[1]*cos(lat)+vec[2]*sin(lat)])

    ha = atan2(-ra_vec[1],ra_vec[0])
    de = asin(ra_vec[2])
    return ha, de


def intersect(org, dir, radius):
    # compute the intersection between a half-line and a sphere
    # the line is defined by the origin and a direction:
    #   x = org[0] + t * dir[0]
    #   y = org[1] + t * dir[1]     for all t>0
    #   z = org[2] + t * dir[2]
    # the sphere is centered in 0 0 0:
    #   x² + y² + z² = radius²
    # Thus the quadratic equation:
    # t² * (dir[0]² + dir[1]² + dir[2]²) + t * (2*org[0]*dir[0] + 2*org[1]*dir[1] + 2*org[2]*dir[2]) + (org[0]² + org[1]² + org[2]² - radius²) = 0

    a = dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]
    b = 2*org[0]*dir[0] + 2*org[1]*dir[1] + 2*org[2]*dir[2]
    c = org[0]*org[0] + org[1]*org[1] + org[2]*org[2] - radius*radius
    delta = b*b - 4*a*c
    t = (-b + sqrt(delta)) / (2*a) # we are only interested in the positive solution
    return t


def print_vec(vec, name=""):
    print("{:} ({:0.2f} {:0.2f} {:0.2f})".format(name, vec[0], vec[1], vec[2]))

def mod(val,center=0):
    # return result modulo 2 pi within center-pi and center+pi
    return ((val+pi-center) % (2*pi)) - pi+center

def compute_azimuth(ha, de, lat, mount_origin, dome_radius, opening_width, scope_diameter, scope_offset):
    # compute cupola azimuth based on ha (hour angle) and de (declination).
    # all angles in rad
    # ha: hour angle, 0 at meridian, positive to the west
    # de: declination, pi/2 at north pole
    # az: azimuth, 0 at north, pi/2 at east

    nb_rays = 12

    # compute intersection between the center of the scope and the dome, then azimuth and tolerance
    scope_dir = rotate(np.array([1, 0, 0]), lat, ha, de)
    az_scope = atan2(scope_dir[0],scope_dir[1])
    el_scope = asin(scope_dir[2])

    scope_origin = mount_origin + rotate(np.array([0, scope_offset[0], 0]), lat, ha, de)
    t = intersect(scope_origin, scope_dir, dome_radius)
    intersection = scope_origin + t * scope_dir
    az_center = atan2(intersection[0],intersection[1])
    #print_vec(scope_origin)
    #print("az_center =",to_deg(az_center))
    if intersection[2]<0:
        az_tol = -180 #below horizon
    elif opening_width / 2 > sqrt(intersection[0] ** 2 + intersection[1] ** 2):
        az_tol = pi / 2  # close to the zenith
    else:
        az_tol = asin(opening_width / 2 / sqrt(intersection[0] ** 2 + intersection[1] ** 2))
    az_min = az_center - az_tol
    az_max = az_center + az_tol

    # repeat for the contour of the scope and compute the intersection of all ranges
    for scope in range(len(scope_offset)):
        for i in range(nb_rays):
            ray_origin = mount_origin + rotate(np.array([0, scope_offset[scope] + scope_diameter[scope]/2*sin(i*2*pi/nb_rays), scope_diameter[scope]/2*cos(i*2*pi/nb_rays)]), lat, ha, de)
            t = intersect(ray_origin, scope_dir, dome_radius)
            intersection = ray_origin + t * scope_dir
            az = atan2(intersection[0],intersection[1])
            az = mod(az, az_center)
            if intersection[2] < 0:
                az_tol = -180  # below horizon
            elif opening_width/2 > sqrt(intersection[0]**2+intersection[1]**2):
                az_tol = pi/2 # close to the zenith
            else:
                az_tol = asin(opening_width/2 / sqrt(intersection[0]**2+intersection[1]**2))

            az_min = max(az_min, az - az_tol)
            az_max = min(az_max, az + az_tol)
    #print("az_min =", to_deg(az_min))
    #print("az_max =", to_deg(az_max))
    az_opt = mod((az_max + az_min)/2,pi)
    tolerance = (az_max - az_min)/2
    #tolerance = max(tolerance,to_rad(0))
    #tolerance = max(min(tolerance,to_rad(2)),to_rad(-0))
    #print(to_deg(az_opt),to_deg(tolerance))
    return az_opt, tolerance, az_scope, el_scope


def plot_accessible_range():
    # position of the intersection between mount axes in the dome frame (+X=East +Y=North +Z=zenith)
    mount_origin = np.array([0, 100, 0])

    dome_radius = 3200  # the center of the dome is 0,0,0
    opening_width = 1000

    # offset between the mount origin and the scope: positive to the east when the mount is pointing the south
    scope_offset = [300, -300]
    scope_diameter = [400, 150]
    latitude = to_rad(43.75)

    az_list = np.radians(np.linspace(-180, 180, 180))
    el_list = np.radians(np.linspace(0, 90, 45))

    el_grid, az_grid = np.meshgrid(el_list, az_list)
    ha_grid = np.zeros(el_grid.shape)
    de_grid = np.zeros(el_grid.shape)
    az_opt = np.zeros(el_grid.shape)
    tol = np.zeros(el_grid.shape)

    for i in range(az_list.size):
        for j in range(el_list.size):
            ha_grid[i, j], de_grid[i, j] = reverse(az_list[i], el_list[j], latitude)
            az_opt[i, j], tol[i, j], az, el = compute_azimuth(ha_grid[i, j], de_grid[i, j], latitude, mount_origin, dome_radius, opening_width, scope_diameter, scope_offset)
            # print(to_deg(ha_list[i]),to_deg(de_list[j]),to_deg(az[i,j]),to_deg(tol[i,j]))

    # print(tol)
    # -- Plot... ------------------------------------------------
    fig, ax = plt.subplots(subplot_kw=dict(projection='polar'))
    # CS = ax.contourf(ha_grid, np.degrees(de_grid), np.degrees(tol),360)
    # CS = ax.contourf(ha_grid, np.degrees(de_grid), np.degrees(el),360)
    CS = ax.contourf(az_grid, np.degrees(el_grid), np.degrees(tol), 360)
    ax.invert_yaxis()
    ax.set_theta_zero_location("N")
    # ax.invert_xaxis()
    # CS = ax.contourf(az, np.degrees(el), np.degrees(tol),360)
    cbar = fig.colorbar(CS)
    plt.show()





# plot_accessible_range()