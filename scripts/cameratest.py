import picamera
camera = picamera.PiCamera()
camera.capture('testimage.jpg')
camera.close()
