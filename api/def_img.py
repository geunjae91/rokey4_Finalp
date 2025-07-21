## 컴퓨터와 연결된 카메라 확인하고
## 몇번 카메라인지 0~순서대로 확인하시면서 원하는 카메라 설정하신다면
## 페이지에 띄울 수 있게끔 해주는 def입니다. 

## 이하 코드는 app.py에 있어야 합니다. 
## @app.route('/video_feed')
## def video_feed():
##    # 여기서 generate_video_stream()으로 MJPEG 영상 스트리밍
##    return Response(generate_video_stream(),
##                    mimetype='multipart/x-mixed-replace; boundary=frame') 

## 이하 코드는 원하는 html에 있어야 합니다. 
## <img id="webcam" src="{{ url_for('video_feed') }}" width="640" height="480">

import cv2

def generate_video_stream():
    cap = cv2.VideoCapture(2)  # 0번 카메라 (노트북 기본 캠)

    if not cap.isOpened():
        raise RuntimeError("카메라를 열 수 없습니다.")

    while True:
        success, frame = cap.read()
        if not success:
            break

        # 프레임을 JPEG로 인코딩
        _, buffer = cv2.imencode('.jpg', frame)
        frame_bytes = buffer.tobytes()

        # multipart/x-mixed-replace 형식으로 전송
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')