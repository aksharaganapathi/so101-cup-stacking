import cv2
from flask import Flask, Response

CAMERA_INDEX = 1   
HOST_IP = "0.0.0.0" 
PORT = 8090          

app = Flask(__name__)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW) 

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
# ------------------------------------------------

def gen_frames():
    """Video streaming generator function."""
    print("Streaming started...")
    while True:
        ret, frame = cap.read()
        if not ret:
            width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
            height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
            print(f"Error: Failed to read frame.")
            print(f"Camera might not support 1080p. Current res: {width}x{height}")
            break
        
        (flag, encodedImage) = cv2.imencode(".jpg", frame)
        if not flag:
            continue

        yield(b'--frame\r\n'
              b'Content-Type: image/jpeg\r\n\r\n' + bytearray(encodedImage) + b'\r\n')

@app.route("/")
def index():
    """Serves the main page with the video feed."""
    
    return (
        "<html><body>"
        "<h1>Live Camera Feed (1080p)</h1>"
        "<img src='/video_feed' style='width: 90%; max-width: 1920px;'>"
        "</body></html>"
    )

@app.route("/video_feed")
def video_feed():
    """Video streaming route."""
    return Response(
        gen_frames(), 
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

if __name__ == "__main__":
    if not cap.isOpened():
        print(f"Error: Could not open camera at index {CAMERA_INDEX}")
    else:
        width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        print(f"Attempting 1080p. Actual resolution: {width}x{height}")
        
        print(f"Server starting... open http://{HOST_IP}:{PORT}/ in your browser.")
        app.run(host=HOST_IP, port=PORT, debug=False, threaded=True)