All of the following functions are in python, but are being ported to C. Also, check out model/box.xml to find out how to get a working ArUco marker in mujoco!

cart_pendulum.py is an implementation of setting a subcreen for the cart pole. The subscreen shows the top view of the pole looking down at the cart.

diff_drive_tracking.py is similar to opencv_tracking.py except there is a differential drive car that is being controlled to track a moving object.

mujoco_tracking.py is an implementation of the mujoco camera tracking feature on another body in the subscreen.

opencv_tracking.py tracks a moving object with opencv integration for applying a bounding box on the item being tracked.

utils/render_insetscreen.py is a function call for setting subscreens in the main mujoco simulation window. Add cameras as needed in xml files.

utils/get_frame.py is a function call for obtaining the pixels in a frame that will be passed to detect_and_draw_bound.py for further processing.

utils/detect_and_draw_bound.py is a function call that draws a bounding box on (yellow) colored objects. Requires the frame obtained by get_frame.py. The color of the object can be specified.
