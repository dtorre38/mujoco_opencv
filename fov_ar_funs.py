import math

def vertical_fov_from_horizontal(horizontal_fov_deg, aspect_ratio):
    """
    Compute the vertical FOV from the horizontal FOV and the aspect ratio.
    """
    horizontal_fov_rad = math.radians(horizontal_fov_deg)
    vertical_fov_rad = 2 * math.atan(math.tan(horizontal_fov_rad / 2) / aspect_ratio)
    
    return round(math.degrees(vertical_fov_rad), 2)

def horizontal_fov_from_vertical(vertical_fov_deg, aspect_ratio):
    """
    Compute the horizontal FOV from the vertical FOV and the aspect ratio.
    """
    vertical_fov_rad = math.radians(vertical_fov_deg)
    horizontal_fov_rad = 2 * math.atan(math.tan(vertical_fov_rad / 2) * aspect_ratio)
    
    return round(math.degrees(horizontal_fov_rad), 2)


def aspect_ratio_from_fovs(horizontal_fov_deg, vertical_fov_deg):
    """
    Compue the aspect ratio from the FOVs.
    """
    # Convert degrees to radians
    horizontal_fov_rad = math.radians(horizontal_fov_deg)
    vertical_fov_rad = math.radians(vertical_fov_deg)
    
    # Calculate aspect ratio
    aspect_ratio = math.tan(horizontal_fov_rad / 2) / math.tan(vertical_fov_rad / 2)
    
    return round(aspect_ratio, 2)

def aspect_ratio_from_pixels(width, height):
    """
    Compue the aspect ratio from the pixel width and height.
    """
    # Calculate aspect ratio
    aspect_ratio = width / height
    
    return round(aspect_ratio, 2)

# Example usage - VFOV -> HFOV
horizontal_fov_deg = 90  # Horizontal field of view in degrees
aspect_ratio =  640/480   # Aspect ratio (width / height)

vertical_fov_deg = vertical_fov_from_horizontal(horizontal_fov_deg, aspect_ratio)
print(f"Horizontal FoV: {horizontal_fov_deg} degrees")
print(f"Vertical FoV: {vertical_fov_deg} degrees\n")

# Example usage - HFOV -> VFOV
# vertical_fov_deg = 90
aspect_ratio =  640/480   # Aspect ratio (width / height)

horizontal_fov_deg = horizontal_fov_from_vertical(vertical_fov_deg, aspect_ratio)
print(f"Vertical FoV: {vertical_fov_deg} degrees")
print(f"Horizontal FoV: {horizontal_fov_deg} degrees\n")

# Example: HFOV & VFOV to AR
horizontal_fov_deg = 120
vertical_fov_deg = 112.38

aspect_ratio = aspect_ratio_from_fovs(horizontal_fov_deg, vertical_fov_deg)
print(f"Aspect Ratio: {aspect_ratio}")

# Example: Pixel width and height -> AR
width = 928
height = 800
width = 928
height = 800
aspect_ratio = aspect_ratio_from_pixels(width, height)
print(f"Aspect Ratio: {aspect_ratio}")