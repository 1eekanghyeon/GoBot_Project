import websocket
import json

def send_payload():
    ws_url = "ws://127.0.0.1:8080"

    data = {
        "type": "end",
        "from": "wsl",
        "to": "app",
        "payload": "end"
    }

    try:
        # WebSocket 연결
        ws = websocket.WebSocket()
        ws.connect(ws_url)

        # JSON 문자열로 변환 후 전송
        message = json.dumps(data)
        ws.send(message)
        print("Sent:", message)

        # 혹시 서버에서 응답이 오면 출력(옵션)
        try:
            response = ws.recv()
            print("Received:", response)
        except:
            pass

        ws.close()

    except Exception as e:
        print("Error:", e)

if __name__ == "__main__":
    send_payload()
