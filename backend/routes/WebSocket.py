from email import utils
from flask import Flask
from flask_socketio import SocketIO, emit
from frozendict import V2

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")


@app.route("/maps")
def maps_list():
    return {"maps": ["map1", "map2"]}


@socketio.on("map_event")
def handle_message(data):
    print("Received:", data)
    emit("server_reply", "Message received!")


if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000)


