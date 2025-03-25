import cv2
import numpy as np


def draw_borders(image: np.ndarray) -> np.ndarray:
    """Draws a border around the image with #(n_tiles) tiles and #(padding) pixels padding.

    Args:
        image (np.ndarray): Input ArUco marker image.

    Returns:
        np.ndarray: Output image with borders.
    """
    
    height, width = image.shape[:2]

    # Parameters
    n_tiles = 8
    padding = 3
    corner_length = 4
    dx = width // n_tiles
    dy = height // n_tiles

    print(f"height = {height}; width = {width}")
    print(f"dx = {dx}; dy = {dy}")

    # Create a new image with padding
    canvas_height = height + 2 * padding * dy
    canvas_width = width + 2 * padding * dx
    canvas = np.ones((canvas_height, canvas_width, 3), dtype=np.uint8) * 255  # White canvas

    # Draw corners
    cv2.rectangle(canvas, (0, 0), (dx, dy * corner_length), (0, 0, 0), cv2.FILLED)
    cv2.rectangle(canvas, (0, 0), (corner_length * dx, dy), (0, 0, 0), cv2.FILLED)
    cv2.rectangle(canvas, (canvas_width - dx, canvas_height - dy * corner_length), 
                  (canvas_width, canvas_height), (0, 0, 0), cv2.FILLED)
    cv2.rectangle(canvas, (canvas_width - corner_length * dx, canvas_height - dy), 
                  (canvas_width, canvas_height), (0, 0, 0), cv2.FILLED)
    cv2.rectangle(canvas, (canvas_width - dx, 0), 
                  (canvas_width, dy * corner_length), (0, 0, 0), cv2.FILLED)
    cv2.rectangle(canvas, (canvas_width - corner_length * dx, 0), 
                  (canvas_width, dy), (0, 0, 0), cv2.FILLED)
    cv2.rectangle(canvas, (0, canvas_height - dy * corner_length), 
                  (dx, canvas_height), (0, 0, 0), cv2.FILLED)
    cv2.rectangle(canvas, (0, canvas_height - dy), 
                  (corner_length * dx, canvas_height), (0, 0, 0), cv2.FILLED)

    # Place the original image onto the canvas
    canvas[padding * dy:padding * dy + height, padding * dx:padding * dx + width] = image

    return canvas

def main(image_path):
    # Read image
    image = cv2.imread(image_path)

    if image is None:
        print(f"Error: Could not load image {image_path}")
        return

    # Draw borders
    image_with_borders = draw_borders(image)

    # Save the image
    output_image_path = "marker23.png"
    cv2.imwrite(output_image_path, image_with_borders)

    print(f"Output image saved as {output_image_path}")


if __name__ == "__main__":
    import sys
    
    if len(sys.argv) != 2:
        print(f"Usage: {sys.argv[0]} <image path>")
    else:
        main(sys.argv[1])