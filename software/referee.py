from websockets.sync.client import connect
import json

class Referee:
    def __init__(self,ID,IP):
        self.match_in_progress = None
        self.basket_color = None
        self.address = IP
        self.robot_id = ID
        self.connected = None
    def listen(self):
        while True:
            try:
                with connect(self.address) as websocket:
                    self.connected = True
                    message = websocket.recv()
                    data = json.loads(message)
                    self.match_in_progress = True if data["signal"] == "start" else False
                    if self.match_in_progress:
                        self.basket_color = data["baskets"][data["targets"].index(self.robot_id)]
            # Don't know what error is thrown when server is closed - Acting on everything
            except:
                self.connected = False
                