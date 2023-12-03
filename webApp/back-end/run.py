from flask_cors import CORS
from services import socket_service,robot_simulation
import flask 
from controllers import routes 

app = flask.Flask(__name__)
app.register_blueprint(routes.main)
app.config["DEBUG"] = True
CORS(app)
socket_service.socketio.init_app(app, cors_allowed_origins="*")

if __name__ == "__main__":
    socket_service.socketio.run(app, host='0.0.0.0', port=8000)
