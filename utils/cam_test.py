import cv2
import numpy as np

CAM_INDEX = 0


def find_red_cube_centroid(frame_bgr):
	hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

	lower_red1 = np.array([0, 100, 80])
	upper_red1 = np.array([10, 255, 255])
	lower_red2 = np.array([160, 100, 80])
	upper_red2 = np.array([179, 255, 255])

	mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
	mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
	mask = cv2.bitwise_or(mask1, mask2)

	kernel = np.ones((5, 5), np.uint8)
	mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
	mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

	contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
	if not contours:
		return None, mask

	c = max(contours, key=cv2.contourArea)
	area = cv2.contourArea(c)
	if area < 100:
		return None, mask

	M = cv2.moments(c)
	if M["m00"] == 0:
		return None, mask

	cx = int(M["m10"] / M["m00"])
	cy = int(M["m01"] / M["m00"])
	return (cx, cy), mask


def main():
	cap = cv2.VideoCapture(CAM_INDEX)
	if not cap.isOpened():
		print(f"Could not open camera index {CAM_INDEX}")
		return

	print("Camera opened. Press 'q' to quit.")

	while True:
		ret, frame = cap.read()
		if not ret:
			print("Failed to grab frame")
			break

		(centroid, mask) = find_red_cube_centroid(frame)

		display = frame.copy()
		h, w = display.shape[:2]

		center = (w // 2, h // 2)
		cv2.circle(display, center, 5, (255, 255, 255), 1)

		if centroid is not None:
			u, v = centroid
			cv2.circle(display, (u, v), 6, (0, 255, 0), -1)
			cv2.putText(display, f"({u},{v})", (u + 10, v - 10),
						cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
			print(f"Red cube at pixel: ({u}, {v})")
		else:
			cv2.putText(display, "No red cube detected", (10, 30),
						cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

		cv2.imshow("Camera", display)
		cv2.imshow("Red mask", mask)

		key = cv2.waitKey(1) & 0xFF
		if key == ord("q"):
			break

	cap.release()
	cv2.destroyAllWindows()


if __name__ == "__main__":
	main()