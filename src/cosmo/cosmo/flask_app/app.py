from flask import Flask, Response, render_template, request, url_for
# from picamera2 import Picamera2
import cv2
import time
import threading
import os
from pathlib import Path 

def rel_path(filepath):
    return Path(os.path.dirname(__file__), filepath)



qr_codes = []

app = Flask(__name__)

# Configure camera

# camera = Picamera2()
# camera.configure(camera.create_preview_configuration(raw={"size":(1640,1232)}, main={"format": 'RGB888', "size": (640, 480)}))
# camera.start()

# Temporary video substitute

camera = cv2.VideoCapture(str(rel_path("./Testing/stockvideo2.mp4")))

# Initialize QR code detector
detector = cv2.QRCodeDetector()

# MediaMTX streaming function
def generate_frames_mtx(fps=15, width=640, height=480):
    ip = 'rtsp://192.168.1.211'
    port = 8554
    print("I'M RUNNING NOW")
    
    global qr_codes

    # Define the GStreamer pipeline
    out = cv2.VideoWriter('appsrc ! videoconvert' + \
        ' ! video/x-raw,format=I420' + \
        ' ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=' + str(fps * 2) + \
        ' ! video/x-h264,profile=baseline' + \
        f' ! rtspclientsink location={ip}:{port}/mystream',
        cv2.CAP_GSTREAMER, 0, fps, (width, height), True)
    if not out.isOpened():
        raise Exception(f"ERROR: Can't open video writer, {cv2.getBuildInformation()}")

    frame_num = 0

    #Capture frame and look for QRs
    while True:
        # frame = camera.capture_array()

        camera.set(cv2.CAP_PROP_POS_FRAMES, frame_num)
        res, frame = camera.read()
        frame_num += 1

        # Apply your OpenCV processing here
        data, bbox, _ = detector.detectAndDecode(frame)

        if len(data)>0 and bbox is not None and len(bbox)>0:
            bbox = bbox.astype(int)

            #Blue Box around QR code
            for i in range(len(bbox[0])):
                cv2.line(frame, tuple(bbox[0][i]), tuple(bbox[0][(i+1) % 4]), (255, 0, 0), 2)

        # Add new QR code data only if it's different from the last scanned
        if data and (len(qr_codes) == 0 or data != qr_codes[-1]):
            qr_codes.append(data)
            print("QR Code Found:", data)


        out.write(frame)


# MJPEG streaming function (old)
def generate_frames():
    global qr_codes
    while True:
        frame = camera.capture_array()
        
        ###### QR Code Decoding #######
        data, bbox, _ = detector.detectAndDecode(frame)

        if len(data)>0 and bbox is not None and len(bbox)>0:
            
            bbox = bbox.astype(int)
            print(data)

            for i in range(len(bbox[0])):
                cv2.line(frame, tuple(bbox[0][i]), tuple(bbox[0][(i+1) % 4]), (255, 0, 0), 2)

            # Add new QR code data only if it's different from the last scanned
            if data and (len(qr_codes) == 0 or data != qr_codes[-1]):
                qr_codes.append(data)
                print("QR Code Found:", data)
        ###############################
        
        ret, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

# Enpoint for streaming MJPEG frames. Uncomment to use. 
# @app.route('/video_feed')
# def video_feed():
#     return Response(generate_frames(), headers={"Cache-Control": "no-cache"}, mimetype='multipart/x-mixed-replace; boundary=frame')


# SSE endpoint to stream new QR code events
@app.route('/qr_feed')
def qr_feed():
    def event_stream():
        last_index = 0
        while True:
            if len(qr_codes) > last_index:
                # Send any new QR codes that have been added
                code = qr_codes[-1]
                # 2 \n signal data is an event!
                yield f"data: {code}\n\n"
                last_index = len(qr_codes)
            time.sleep(0.5)
    return Response(event_stream(), mimetype="text/event-stream")


# Endpoint to handle key control input
@app.route('/key_control', methods=['POST'])
def key_control():
    data = request.get_json()
    key = data.get('key')
    # Here you can add your code to send commands to your Raspberry Pi based on the key
    print(f"Received key: {key}")
    # For example, you might call a function that controls motors
    return {} #{'status': 'success', 'key': key}

# Endpoint to handle controller input
@app.route('/controller_control', methods=['POST'])
def controller_control():
    data = request.get_json()
    R2buttons = data.get('R2')

    print(f"Received R2 buttons: {R2buttons}")

    return{}



def start_server(_host='0.0.0.0', _port=5000):
    # Start generate_frames_mtx loop in a separate thread, since app.run() is blocking
    threading.Thread(target=generate_frames_mtx, daemon=True).start()

    #Start Flask app
    app.run(host=_host, port=_port)


# Or comment out below and use:
# gunicorn -k gevent -w 1 app:app
# In command line
if __name__ == '__main__':
    # Start generate_frames_mtx loop in a separate thread, since app.run() is blocking
    threading.Thread(target=generate_frames_mtx, daemon=True).start()

    #Start Flask app
    app.run(host='0.0.0.0', port=5000)



    
