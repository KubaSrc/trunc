import cv2
import numpy as np
import apriltag as april

#load image into program
imageB = cv2.imread('images/CameraB.jpg', cv2.IMREAD_GRAYSCALE)
imageL2 = cv2.imread('images/CameraL2.jpg', cv2.IMREAD_GRAYSCALE)
imageR = cv2.imread('images/CameraR.jpg', cv2.IMREAD_GRAYSCALE)

#create a detector
options = april.DetectorOptions(families='tag16h5')
detector = april.Detector(options)

# Blur the image for better edge detection
img_blurB = cv2.GaussianBlur(imageB, (7, 7), 0)
img_blurL2 = cv2.GaussianBlur(imageL2, (7, 7), 0)
img_blurR = cv2.GaussianBlur(imageR, (7, 7), 0)

_, img_blurL2 = cv2.threshold(img_blurL2, 127, 255, cv2.THRESH_BINARY)

# # Display the original and edge-detected images

cv2.imshow('img_blurL2', img_blurL2)
#cv2.imshow('Original Image', imageL2)
cv2.waitKey(0)  # Wait for a key press to close the windows

#detect any apriltags
tags = detector.detect(img_blurB)
print(tags)

# Process each detected tag
for tag in tags:
    # Get the detected corners
    corners = np.array(tag.corners)

    # Calculate the centroid (average) of the corners
    centroid_x = np.mean(corners[:, 0])
    centroid_y = np.mean(corners[:, 1])

    # Assume a fixed tag size (you can adjust this based on your specific tags)
    tag_size = 0.1  # Example: 10 cm

    # Compute the position (X, Y, Z) relative to the camera
    pose_x = (centroid_x - imageL2.shape[1] / 2) * tag_size
    pose_y = (centroid_y - imageL2.shape[0] / 2) * tag_size
    pose_z = tag_size  # Assuming the tag is at a fixed height (e.g., on a flat surface)

    print(f"Tag ID {tag.tag_id}: Position (X, Y, Z) = ({pose_x:.2f}, {pose_y:.2f}, {pose_z:.2f})")

    print(f"Orientation {tag.tag_id}: {tag.homography}")

    #Quaternions: vector with 4 elements
    #set of rotation matrices
    #3 cameras to triangulate

