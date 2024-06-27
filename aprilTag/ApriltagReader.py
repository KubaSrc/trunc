import cv2
import numpy as np
import apriltag as april
from scipy.spatial.transform import Rotation as R


#write script to take images on demand and process the script
#find out how long it takes to do so
#Look into calibrating the cameras
#undistort the images


########
#MODIFY THE NAME FOR CAMERAL2 TO CAMERAL
########
files = ['images/CameraB.jpg', 'images/CameraL2.jpg', 'images/CameraR.jpg']

# create a detector
options = april.DetectorOptions(families='tag16h5')
detector = april.Detector(options)

processed = []
finalP =[]
finalQ =[]

for file in files:
    #load image into program
    image = cv2.imread(file, cv2.IMREAD_GRAYSCALE)

    #old code vvv
    # imageL2 = cv2.imread('images/CameraL2.jpg', cv2.IMREAD_GRAYSCALE)
    # imageR = cv2.imread('images/CameraR.jpg', cv2.IMREAD_GRAYSCALE)

    # Blur the image for better edge detection
    img_blur = cv2.GaussianBlur(image, (7, 7), 0)
    #old code vvv
    # img_blurL2 = cv2.GaussianBlur(imageL2, (7, 7), 3)
    # img_blurR = cv2.GaussianBlur(imageR, (7, 7), 3)

    _, img_blur = cv2.threshold(img_blur, 127, 255, cv2.THRESH_BINARY)

    # # Display the original and edge-detected images
    #cv2.imshow('Original Image', imageL2)
    # cv2.imshow('img_blurL2', img_blur)
    # cv2.waitKey(0)  # Wait for a key press to close the windows

    #detect any apriltags
    processed.append(detector.detect(img_blur))

# Process each detected tag
for tags in processed:
    for tag in tags:
        # Get the detected corners
        corners = np.array(tag.corners)

        # Calculate the centroid (average) of the corners
        centroid_x = np.mean(corners[:, 0])
        centroid_y = np.mean(corners[:, 1])

        # Assume a fixed tag size (you can adjust this based on your specific tags)
        tag_size = 0.02  #20 mm

        # Compute the position (X, Y, Z) relative to the camera
        pose_x = (centroid_x - image.shape[1] / 2) * tag_size
        pose_y = (centroid_y - image.shape[0] / 2) * tag_size
        pose_z = tag_size  # Assuming the tag is at a fixed height (e.g., on a flat surface)

        print(f"Tag ID {tag.tag_id}: Position (X, Y, Z) = ({pose_x:.2f}, {pose_y:.2f}, {pose_z:.2f})")
        ##Debugging
        #finalP.append(f"Tag ID {tag.tag_id}: Position (X, Y, Z) = ({pose_x:.2f}, {pose_y:.2f}, {pose_z:.2f})")

        #retrieve the homography
        hom = tag.homography
        #normalize it
        hom /= hom[2,2]

        # Extract the rotation matrix from the homography matrix
        r1 = hom[:, 0]
        r2 = hom[:, 1]
        r3 = np.cross(r1, r2)
        rotation_matrix = np.stack((r1, r2, r3), axis=1)

        #Turn the matrix into a quaternion
        rotation = R.from_matrix(rotation_matrix)
        quaternion = rotation.as_quat()

        print(f"Orientation {tag.tag_id}: {quaternion}")

        ##Debugging
        #finalQ.append(f"Orientation {tag.tag_id}: {quaternion}")

#print(finalP)
#print(finalQ)



