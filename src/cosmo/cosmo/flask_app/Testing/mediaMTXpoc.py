import cv2
import time 
from picamera2 import Picamera2

fps = 15
width = 640
height = 480

# Define the GStreamer pipeline
out = cv2.VideoWriter('appsrc ! videoconvert' + \
    ' ! video/x-raw,format=I420' + \
    ' ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=' + str(fps * 2) + \
    ' ! video/x-h264,profile=baseline' + \
    ' ! rtspclientsink location=rtsp://192.168.1.211:8554/mystream',
    cv2.CAP_GSTREAMER, 0, fps, (width, height), True)
if not out.isOpened():
    raise Exception("can't open video writer")

# cap = cv2.VideoCapture(0)
camera = Picamera2()
camera.configure(camera.create_preview_configuration(
    raw={"size":(1640,1232)}, main={"format": 'RGB888', "size": (640, 480)}))
camera.start()

detector = cv2.QRCodeDetector()


while True:
    # ret, frame = cap.read()
    frame = camera.capture_array()
    
    # if not ret:
    #     break

    # Apply your OpenCV processing here
    data, bbox, _ = detector.detectAndDecode(frame)

    if len(data)>0 and bbox is not None and len(bbox)>0:
        bbox = bbox.astype(int)
        print(data)
        #Blue Box around QR code
        for i in range(len(bbox[0])):
            cv2.line(frame, tuple(bbox[0][i]), tuple(bbox[0][(i+1) % 4]), (255, 0, 0), 2)

    # Add new QR code data only if it's different from the last scanned
    # if data and (len(qr_codes) == 0 or data != qr_codes[-1]):
    #     qr_codes.append(data)
    #     print("QR Code Found:", data)


    out.write(frame)

# cap.release()
out.release()
