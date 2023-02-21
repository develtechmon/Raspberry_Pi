
from flask import Flask, render_template, Response
from threading import Thread
import cv2
import time
import socket
import io
from picamera2 import Picamera2

app = Flask(__name__)

vc = Picamera2()
vc.configure(vc.create_preview_configuration(main={"format": 'XRGB8888', "size": (1024, 768)}))
vc.start()

time.sleep(1.0)

@app.route('/')
def index():
    """Video streaming"""
    return render_template('index.html')

def gen(vc):
    """Video streaming generator function."""
    #yield b'--frame\r\n'
    while True:
        frame = vc.capture_array()
        ret,jpeg = cv2.imencode('.jpg',frame)
        frame = jpeg.tobytes()
        yield b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n--frame\r\n'
        
@app.route('/video_feed')
def video_feed():
    #global vc
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen(vc),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
        app.run(host='192.168.195.35', port=80, threaded=True)
               
