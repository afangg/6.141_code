import cv2
import imutils
import numpy as np
import pdb
import sys

#################### X-Y CONVENTIONS #########################
# 0,0  X  > > > > >
#
#  Y
# 
#  v  This is the image. Y increases downwards, X increases rightwards
#  v  Please return bounding boxes as ((xmin, ymin), (xmax, ymax))
#  v
#  v
#  v
###############################################################
# ORANGE_LOWER_BOUND = np.array([5, 20, 20], np.uint8)
# ORANGE_UPPER_BOUND = np.array([18, 255, 255], np.uint8)

#THRESH_LOW = np.array([0, 40, 135], np.uint8)
#THRESH_HIGH = np.array([19, 255, 255], np.uint8)
THRESH_LOW = np.array([0, 59, 162], np.uint8)
THRESH_HIGH = np.array([11, 255, 255], np.uint8)


# KERNEL_OPEN = np.ones((5, 5))
# KERNEL_CLOSE = np.ones((3, 3))
KERNEL = np.ones((3, 3), np.uint8)

# Set to 1 to show debugging images
DEBUG = 0

def image_print(img):
	"""
	Helper function to print out images, for debugging. Pass them in as a list.
	Press any key to continue.
	"""
	cv2.imshow("image", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()

def restrict_line_view(img):
    """
    Takes a cv2 image object and blacks out top and bottom portions to
    restrict the view in way that enables line following.

    img: opencv image object.
    returns: img copy with particular portions blacked out.
    """
    h, w, _  = img.shape
    # Tunable
    box_1 = ((0, 0), (w, int((2.5 / 4) * h)))
    box_2 = ((0, int((6 / 8) * h)), (w, h))
    first_box = cv2.rectangle(img, box_1[0], box_1[1], (0, 0, 0), -1)
    restricted_img = cv2.rectangle(first_box, box_2[0], box_2[1], (0, 0, 0), -1)

    return restricted_img

def cd_color_segmentation(img, template):
	"""
	Implement the cone detection using color segmentation algorithm
	Input:
		img: np.3darray; the input image with a cone to be detected. BGR.
		template_file_path; Not required, but can optionally be used to automate setting hue filter values.
	Return:
		bbox: ((x1, y1), (x2, y2)); the bounding box of the cone, unit in px
				(x1, y1) is the bottom left of the bbox and (x2, y2) is the top right of the bbox
	"""
	bounding_box = None
	restricted_img = restrict_line_view(img.copy())

	# Convert color space to HSV
	imgHSV = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
	imgThresh = cv2.inRange(imgHSV, THRESH_LOW, THRESH_HIGH)
	# Some open and close morphology
	maskOpen = cv2.morphologyEx(imgThresh, cv2.MORPH_OPEN, KERNEL)
	maskClose = cv2.morphologyEx(maskOpen, cv2.MORPH_CLOSE, KERNEL)
	# Erosion and Dilation
	imgEroded = cv2.erode(maskClose, KERNEL, iterations=1)
	imgDilated = cv2.dilate(imgEroded, KERNEL, iterations=1)
	# Canny algorithm for edge detection
	imgCanny = cv2.Canny(imgDilated, 80, 100)
	# Find contours as a first step to find the bounding box
	_, contours, _ = cv2.findContours(imgCanny.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

	# We will assume that other orange objects might appear in the image, and
	# we will assume that the cone is the largest of those orange objects.
	# Therefore, it should have the biggest area (h * w).
	try:
		x_0, y_0, w_max, h_max = max([cv2.boundingRect(cont) for cont in contours],
									 key=lambda x: x[2] * x[3])
		bounding_box = ((x_0, y_0), (x_0 + w_max, y_0 + h_max))
	except:
		print("No cone found!")
		bounding_box = ((0, 0), (0, 0))

	# For debugging
	if DEBUG:
		# cv2.drawContours(img, contours, -1, (255, 0, 0), 2)
		cv2.rectangle(img, bounding_box[0], bounding_box[1], (0, 0, 255), 2)
		image_print(restricted_img)
		image_print(imgThresh)
		image_print(maskOpen)
		image_print(maskClose)
		image_print(imgEroded)
		image_print(imgDilated)
		image_print(imgCanny)
		image_print(img)

	########### YOUR CODE ENDS HERE ###########

	# Return bounding box
	return bounding_box

if __name__ == "__main__":
	args = sys.argv
	if (len(args) > 2):
		DEBUG = 1 if args[1] == '1' else 0
		img_file = args[2]
		box = cd_color_segmentation(cv2.imread(img_file), None)
		print(box)
	else:
		print("Run: python color_segmentation.py DEBUG img_file")
