import numpy as np
import cv2
import csv


def nothing(x):
    pass

cap = cv2.VideoCapture(0)

cv2.namedWindow('Frame')

with open('hsv.csv', 'r', encoding='utf-8') as f:
    rows = csv.reader(f)
    hsv = []
    for row in rows:
        hsv.append(row)
lower = {}
upper = {}
for row in hsv:
    lower[row[0]] = np.array(row[1:4]).astype(np.uint8)
    upper[row[0]] = np.array(row[4:7]).astype(np.uint8)
    
colors = ['black', 'blue', 'green', 'red']
index = 0
color = colors[index]
cv2.createTrackbar("L-H", "Frame", lower[color][0], 180, nothing)
cv2.createTrackbar("L-S", "Frame", lower[color][1], 255, nothing)
cv2.createTrackbar("L-V", "Frame", lower[color][2], 255, nothing)
cv2.createTrackbar("U-H", "Frame", upper[color][0], 180, nothing)
cv2.createTrackbar("U-S", "Frame", upper[color][1], 255, nothing)
cv2.createTrackbar("U-V", "Frame", upper[color][2], 255, nothing)

print('Press c to change color')
while True:
    _, frame = cap.read()
    frame = cv2.resize(frame, (300, 300))
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    l_h = cv2.getTrackbarPos("L-H", "Frame")
    l_s = cv2.getTrackbarPos("L-S", "Frame")
    l_v = cv2.getTrackbarPos("L-V", "Frame")
    u_h = cv2.getTrackbarPos("U-H", "Frame")
    u_s = cv2.getTrackbarPos("U-S", "Frame")
    u_v = cv2.getTrackbarPos("U-V", "Frame")
    
    
    if l_h <= u_h:
        l = np.array([l_h, l_s, l_v])
        u = np.array([u_h, u_s, u_v])
        mask = cv2.inRange(hsv, l, u)
    else:
        l = np.array([l_h, l_s, l_v])
        u = np.array([u_h, u_s, u_v])
        mask_1 = cv2.inRange(hsv, np.array([0, l_s, l_v]), u)
        mask_2 = cv2.inRange(hsv, l, np.array([180, u_s, u_v]))

        mask = cv2.bitwise_or(mask_1, mask_2)
    
    kernel = np.ones((7, 7), np.uint8)
    mask = cv2.dilate(mask, kernel)
    # Contours detection
    _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for cnt in contours:
        area = cv2.contourArea(cnt)
        approx = cv2.approxPolyDP(cnt, 0.02*cv2.arcLength(cnt, True), True)
        x = approx.ravel()[0]
        y = approx.ravel()[1]
        M = cv2.moments(cnt)
        cv2.drawContours(frame, [approx], 0, (255, 255, 255), 3)
        
    cv2.putText(frame, color, (5, 30), cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.imshow('Frame', frame)
    cv2.imshow('Mask', mask)
    
    key = cv2.waitKey(1)
    if key == 27:
        break
    elif key == ord('c'):
        lower[color][0] = l_h
        lower[color][1] = l_s
        lower[color][2] = l_v
        upper[color][0] = u_h
        upper[color][1] = u_s
        upper[color][2] = u_v
        index += 1
        try:
            color = colors[index]
        except:
            break
        cv2.setTrackbarPos('L-H', 'Frame', lower[color][0])
        cv2.setTrackbarPos('L-S', 'Frame', lower[color][1])
        cv2.setTrackbarPos('L-V', 'Frame', lower[color][2])
        cv2.setTrackbarPos('U-H', 'Frame', upper[color][0])
        cv2.setTrackbarPos('U-S', 'Frame', upper[color][1])
        cv2.setTrackbarPos('U-V', 'Frame', upper[color][2])
        
    
cap.release()
cv2.destroyAllWindows()

print('Save parameters? (y/n)')
save = input()
while (save != 'y') and (save != 'n'):
    print('Save parameters? (y/n)')
    save = input()
if save == 'y':
    print('Parameters are saved')
    with open('hsv.csv', 'w', encoding='utf-8') as f:
        w = csv.writer(f)
        for color in colors:
            w.writerow([color]+list(lower[color])+list(upper[color]))
elif save == 'n':
    print('Parameters are not saved')
    pass