from PIL import Image
import numpy as np
from rtree import index
from scipy.spatial import cKDTree
import math

def calculate_angle(A, B, C):
    # Calculate the lengths of the sides of the triangle
    a = math.dist(B, C)
    b = math.dist(A, C)
    c = math.dist(A, B)
    
    # Calculate the angle between sides 'a' and 'b' using the law of cosines
    cos_angle = (b**2 + c**2 - a**2) / (2 * b * c)
    
    # Calculate the angle in radians
    angle_rad = math.acos(cos_angle)
    
    # Convert the angle to degrees
    #angle_deg = math.degrees(angle_rad)
    
    return angle_rad

def coords2pixel(coords, im, tl_bl_br):
    tl = tl_bl_br[0]
    bl = tl_bl_br[1]
    br = tl_bl_br[2]
    x_alligned = bl + [1,0]

    img_angle = calculate_angle(bl, x_alligned, tl)
    
    if not np.isclose(img_angle, math.pi/2, rtol=1e-5, atol=1e-5):
        rot_angle = - img_angle + math.pi/2
        R2 = np.array([[math.cos(rot_angle), -math.sin(rot_angle)],
                       [math.sin(rot_angle),  math.cos(rot_angle)]])
        #R3 = np.eye(3)
        #R3[:2,:2] = R2

        coords = (R2 @ coords.T).T
        tl = (R2 @ tl.T).T
        br = (R2 @ br.T).T

    im_bounds = [tl[0], br[1], br[0], tl[1]]

    w, h = im.size
    w_step = (im_bounds[2] - im_bounds[0])/w
    h_step = (im_bounds[3] - im_bounds[1])/h

    pix_coords = np.empty(coords.shape)

    pix_coords[:,1] = coords[:,0] - im_bounds[0]
    pix_coords[:,0] = - coords[:,1] + im_bounds[1]

    pix_coords[:,0] /= w_step
    pix_coords[:,1] /= h_step

    pix_coords = pix_coords.astype(int)

    return pix_coords

def find_nearest_pixel_color(im, im_array, tl_bl_br, coords):
    pix_coords = coords2pixel(coords ,im, tl_bl_br)

    pix_rgb = im_array[pix_coords[:,0], pix_coords[:,1]]

    return pix_rgb

def img2rgb(im_path, tl_bl_br, coords):
    im = Image.open(im_path)
    im = im.convert('RGB')
    im_array = np.array(im)

    rgb_colors = find_nearest_pixel_color(im, im_array, tl_bl_br, coords) 

    return rgb_colors

# Example usage
if __name__ == "__main__":
    im_path = '/home/aherold/ws/src/cuzk_tools/images/orto.png'
    coords = np.array([[1000,2000],[1500,3000],[3200,1000]])
    im_bounds = [500,500,5500,5500]

    rgb_colors = img2rgb(im_path, im_bounds, coords)
    print(rgb_colors)
