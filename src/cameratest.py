import picamera
import picamera.array
import config as g

camera =  picamera.PiCamera()
image_shape = (300, 300)
camera.resolution = g.image_shape
camera.capture(g.image_path)
camera.close()