import numpy as np
import cv2
import matplotlib.image as mpimg

# Identify Navigable pixels 
def color_Navig_thresh(img, rgb_thresh):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] > rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
	
# Identify Rock Samples
def color_Rock_thresh(img, rgb_thresh):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] > rgb_thresh[0]) \
                & (img[:,:,1] > rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select

	# Identify FOV of Camera
def color_FOV_thresh(img, rgb_thresh):
    # Create an array of zeros same xy size as img, but single channel
    color_select = np.zeros_like(img[:,:,0])
    # Require that each pixel be above all three threshold values in RGB
    # above_thresh will now contain a boolean array with "True"
    # where threshold was met
    above_thresh = (img[:,:,0] < rgb_thresh[0]) \
                & (img[:,:,1] < rgb_thresh[1]) \
                & (img[:,:,2] < rgb_thresh[2])
    # Index the array of zeros with the boolean array and set to 1
    color_select[above_thresh] = 1
    # Return the binary image
    return color_select
	
# Define a function to convert from image coords to rover coords
def rover_coords(binary_img):
    # Identify nonzero pixels
    ypos, xpos = binary_img.nonzero()
    # Calculate pixel positions with reference to the rover position being at the 
    # center bottom of the image.  
    x_pixel = -(ypos - binary_img.shape[0]).astype(np.float)
    y_pixel = -(xpos - binary_img.shape[1]/2 ).astype(np.float)
    return x_pixel, y_pixel


# Define a function to convert to radial coords in rover space
def to_polar_coords(x_pixel, y_pixel):
    # Convert (x_pixel, y_pixel) to (distance, angle) 
    # in polar coordinates in rover space
    # Calculate distance to each pixel
    dist = np.sqrt(x_pixel**2 + y_pixel**2)
    # Calculate angle away from vertical for each pixel
    angles = np.arctan2(y_pixel, x_pixel)
    return dist, angles

# Define a function to map rover space pixels to world space
	# Rotational Matrix
def rotate_pix(xpix, ypix, yaw):
    # Convert yaw to radians
    yaw_rad = yaw * np.pi / 180
    xpix_rotated = (xpix * np.cos(yaw_rad)) - (ypix * np.sin(yaw_rad))
                            
    ypix_rotated = (xpix * np.sin(yaw_rad)) + (ypix * np.cos(yaw_rad))
    # Return the result  
    return xpix_rotated, ypix_rotated

	# Translational Matrix
def translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale): 
    # Apply a scaling and a translation
    xpix_translated = (xpix_rot / scale) + xpos
    ypix_translated = (ypix_rot / scale) + ypos
    # Return the result  
    return xpix_translated, ypix_translated


# Define a function to apply rotation and translation (and clipping)

def pix_to_world(xpix, ypix, xpos, ypos, yaw, world_size, scale):
    # Apply rotation
    xpix_rot, ypix_rot = rotate_pix(xpix, ypix, yaw)
    # Apply translation
    xpix_tran, ypix_tran = translate_pix(xpix_rot, ypix_rot, xpos, ypos, scale)
    # Perform rotation, translation and clipping all at once
    x_pix_world = np.clip(np.int_(xpix_tran), 0, world_size - 1)
    y_pix_world = np.clip(np.int_(ypix_tran), 0, world_size - 1)
    # Return the result
    return x_pix_world, y_pix_world

# Define a function to perform a perspective transform
def perspect_transform(img, src, dst):
           
    M = cv2.getPerspectiveTransform(src, dst)
    warped = cv2.warpPerspective(img, M, (img.shape[1], img.shape[0]))# keep same size as input image
    
    return warped


# Update Rover State
def perception_step(Rover):
    
	#Loading Rover.img with another variable name
    image = Rover.img
    
	#Source and destination points for perspective transform
    dst_size = 5 
    bottom_offset = 6
    source = np.float32([[14, 140], [301 ,140],[200, 96], [118, 96]])
    destination = np.float32([[image.shape[1]/2 - dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - bottom_offset],
                  [image.shape[1]/2 + dst_size, image.shape[0] - 2*dst_size - bottom_offset], 
                  [image.shape[1]/2 - dst_size, image.shape[0] - 2*dst_size - bottom_offset],
                  ])
    
	#Perspective transform
    warped = perspect_transform(image, source, destination)
    
	#Color threshold to identify navigable terrain/obstacles/rock samples
    threshed_Navig = color_Navig_thresh(warped, rgb_thresh=(160,160,160))
    threshed_Rock = color_Rock_thresh(warped, rgb_thresh=(110,110,50))
    threshed_FOV=color_FOV_thresh(warped, rgb_thresh=(1,1,1))
    
    threshed_Obstacle=np.zeros_like(threshed_Navig)
    idx_nav=threshed_Navig==0
    threshed_Obstacle[idx_nav]=1
    idx_fov=threshed_FOV>0 
    threshed_Obstacle[idx_fov]=0
    idx_rock=threshed_Rock>0 
    threshed_Obstacle[idx_rock]=0
	
    #Updating Rover.vision_image
    Rover.vision_image[:,:,0] = threshed_Obstacle*255
    Rover.vision_image[:,:,2] = threshed_Navig*255
	
    #Map image pixel values of navigable terrain to rover-centric coords
    world_size = Rover.worldmap.shape[0]
    scale = 10
    x_pixel, y_pixel=rover_coords(threshed_Navig)
    
	#Navigation angle for Rover in Rover coordinates
    dist, angles = to_polar_coords(x_pixel, y_pixel)
    mean_dir = np.mean(angles)
    Rover.nav_dists = dist
    Rover.nav_angles = angles  

    #Rover-centric pixel values of Navigable terrain to world coordinates
    navigable_x_world, navigable_y_world = pix_to_world(x_pixel, y_pixel, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
    
	#Map image pixel values of obstacles to rover-centric coords
    obs_X, obs_Y= rover_coords(threshed_Obstacle)   
    
	#Rover-centric pixel values of obstacles to world coordinates
    obstacle_x_world,obstacle_y_world = pix_to_world(obs_X, obs_Y, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
	
	#Updating Rover.worldmap
    if (((Rover.pitch < 1.8) | (Rover.pitch > 357.2)) & ((Rover.roll < 1) | (Rover.roll > 359))) & ((Rover.steer < 10) & (Rover.steer > -10)):
        Rover.worldmap[obstacle_y_world, obstacle_x_world, 0] += 1
        Rover.worldmap[navigable_y_world, navigable_x_world, 2] += 10
		
	
    #Finding Rocks based on colour Threshold
    if threshed_Rock.any():
        Rock_X, Rock_Y= rover_coords(threshed_Rock)
        rock_x_world,rock_y_world = pix_to_world(Rock_X, Rock_Y, Rover.pos[0], Rover.pos[1], Rover.yaw, world_size, scale)
        rock_dist, rock_angles = to_polar_coords(rock_x_world, rock_y_world)
        rock_idx = np.argmin(rock_dist)
        rock_x=rock_x_world[rock_idx]
        rock_y=rock_y_world[rock_idx]
        Rover.worldmap[rock_y, rock_x, 1] = 255
        Rover.vision_image[:,:,1] = threshed_Rock*255
        Rover.rock_x=[rock_x]
        Rover.rock_y=[rock_y]
        
    else:
        Rover.vision_image[:,:,1] = 0  
    
    Rover.x_pixel=x_pixel
    Rover.y_pixel=y_pixel
    
    Rover.count=Rover.count+1
	
    print(Rover.count)
    return Rover