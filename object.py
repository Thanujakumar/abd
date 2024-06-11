import cv2
import numpy as np

# Load the template image (the object you want to detect)
template_image_path = r"C:\Users\hp\Downloads\project logo.jpg"
template_image = cv2.imread(template_image_path, cv2.IMREAD_GRAYSCALE)

# Check if the template image is loaded properly
if template_image is None:
    print(f"Error: Template image not found at {template_image_path}")
    exit(1)

# Initialize the ORB detector
orb = cv2.ORB_create()

# Find the keypoints and descriptors with ORB in the template image
keypoints_template, descriptors_template = orb.detectAndCompute(template_image, None)

# Initialize the camera
cap = cv2.VideoCapture(0)

# Check if the camera opened successfully
if not cap.isOpened():
    print("Error: Could not open video device")
    exit(1)

# Create a BFMatcher object with Hamming distance
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

while True:
    # Capture frame-by-frame
    ret, frame = cap.read()

    if not ret:
        print("Error: Failed to capture image")
        break

    # Convert the frame to grayscale
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Find the keypoints and descriptors with ORB in the frame
    keypoints_frame, descriptors_frame = orb.detectAndCompute(gray_frame, None)

    if descriptors_frame is not None and len(descriptors_frame) > 0:
        # Match descriptors between the template image and the frame
        matches = bf.knnMatch(descriptors_template, descriptors_frame, k=2)

        # Apply ratio test to filter good matches
        good_matches = []
        for m, n in matches:
            if m.distance < 0.75 * n.distance:
                good_matches.append(m)

        # Minimum number of good matches required to find the object
        MIN_MATCH_COUNT = 10

        if len(good_matches) >= MIN_MATCH_COUNT:
            src_pts = np.float32([keypoints_template[m.queryIdx].pt for m in good_matches]).reshape(-1, 2)
            dst_pts = np.float32([keypoints_frame[m.trainIdx].pt for m in good_matches]).reshape(-1, 2)

            H, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

            # Get the corners of the template image
            h, w = template_image.shape
            corners = np.float32([[0, 0], [0, h], [w, h], [w, 0]]).reshape(-1, 1, 2)

            if H is not None:
                # Transform the corners to find the location in the frame
                transformed_corners = cv2.perspectiveTransform(corners, H)
                frame = cv2.polylines(frame, [np.int32(transformed_corners)], True, (0, 255, 0), 3, cv2.LINE_AA)
                print("Object Detected!")
            else:
                print("Homography not found. Object not detected.")
        else:
            print(f"Not enough good matches found - {len(good_matches)}/{MIN_MATCH_COUNT}")

    # Draw matches for visualization
    match_img = cv2.drawMatches(template_image, keypoints_template, frame, keypoints_frame, good_matches, None,
                                flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    # Display the frame with detected object
    cv2.imshow('Frame', match_img)

    # Exit if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close OpenCV windows
cap.release()
cv2.destroyAllWindows()
