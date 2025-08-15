###
# This script can be used to generate custom maps for testing
# NOTE: Maps will be inverted 

import cv2
import numpy as np


drawing = False
grid_size = 10  


def click_event(event, x, y, flags, param):
    global drawing, img

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        draw_grid_block(x, y)

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            draw_grid_block(x, y)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False

def draw_grid_block(x, y):
    global img, grid_size

    top_left_x = (x // grid_size) * grid_size
    top_left_y = (y // grid_size) * grid_size
    img[top_left_y:top_left_y+grid_size, top_left_x:top_left_x+grid_size] = 0
    cv2.imshow('image', img)

img = np.ones((500, 500), dtype=np.uint8) 
img = img*255

cv2.imshow('image', img)
cv2.setMouseCallback('image', click_event)

print("Click and hold the mouse to draw. Press 's' to save and 'q' to quit.")

while True:
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        cv2.imwrite('map.png', img)
        print("Image saved as 'map.png'")
    elif key == ord('q'):
        break

cv2.destroyAllWindows()
