import cv2
import numpy as np

leftLineCoordinate = (300, 750)
rightLineCoordinate = (1100, 750)

leftCoordinates = []
rightCoordinates = []


def calculateDistance(first, second):
    tempFirst = np.square(first[0] - second[0])
    tempSecond = np.square(first[1] - second[1])
    return np.sqrt(tempFirst + tempSecond)


def canny_edge_detector(image):
    # Convert the image color to grayscale
    gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

    # Reduce noise from the image
    blur = cv2.GaussianBlur(image, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)
    return canny


def region_of_interest(image):
    height = image.shape[0]
    polygons = np.array([
        [(100, height), (1300, height), (650, 450)]
    ])
    mask = np.zeros_like(image)

    # Fill poly-function deals with multiple polygon
    cv2.fillPoly(mask, polygons, 255)

    # Bitwise operation between canny image and mask image
    masked_image = cv2.bitwise_and(image, mask)
    return masked_image


def draw_lines(img, lines):
    img = np.copy(img)
    blank_image = np.zeros((img.shape[0], img.shape[1], 3), np.uint8)

    if lines is not None:
        for line in lines:
            for x1, y1, x2, y2 in line:
                leftDistance1 = calculateDistance((x1, y1), leftLineCoordinate)
                rightDistance1 = calculateDistance((x1, y1), rightLineCoordinate)

                if leftDistance1 < rightDistance1:
                    leftCoordinates.append((x1, y1))
                    leftCoordinates.append((x2, y2))
                else:
                    rightCoordinates.append((x1, y2))
                    rightCoordinates.append((x2, y2))
                cv2.line(blank_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

    img = cv2.addWeighted(img, 0.8, blank_image, 1, 0.0)

    return img


cap = cv2.VideoCapture("videos/easy.mp4")

while (1):
    _, frame = cap.read()
    hls = cv2.cvtColor(frame, cv2.COLOR_BGR2HLS)
    # white color mask
    lower = np.uint8([0, 200, 0])
    upper = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(hls, lower, upper)
    # yellow color mask
    lower = np.uint8([10, 0, 100])
    upper = np.uint8([40, 255, 255])
    yellow_mask = cv2.inRange(hls, lower, upper)
    # combine the mask

    combo_mask = cv2.bitwise_or(white_mask, yellow_mask)

    final_mask_combo = cv2.bitwise_and(frame, frame, mask=combo_mask)
    cropped_image = region_of_interest(final_mask_combo)

    canny_image = canny_edge_detector(cropped_image)
    combined_lines = cv2.HoughLinesP(canny_image, rho=2, theta=np.pi / 180,
                                     threshold=50, lines=np.array([]), minLineLength=20, maxLineGap=70)

    image_with_lines = draw_lines(frame, combined_lines)

    cv2.imshow("result", image_with_lines)
    print(f'RIGHT : {rightCoordinates}')
    print(f'LEFT : {leftCoordinates}')
    if cv2.waitKey(1) == ord('q'):
        break

cap.release()

cv2.destroyAllWindows()
