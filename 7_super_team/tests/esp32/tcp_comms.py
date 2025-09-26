import socket
import time

class Communications():
    def __init__(self):
        self.IP = "192.168.4.1"
        self.PORT = 1234
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.IP, self.PORT))
        
    def send(self, data: str) -> None:
        formatted_data = f"{data}\n"
        self.sock.sendall(formatted_data.encode())
        
    def receive(self) -> str:
        try:
            data = self.sock.recv(1024)
            return data.decode().strip()
        
        except socket.error as e:
            return f"Error: {e}"
        
    def close(self):
        self.sock.close()

comms = Communications()

try:
    data_to_send = "hello esp32"
    
    while True:
        
        print(f"Sent: {data_to_send}")
        comms.send(data_to_send)
        
        response = comms.receive()
        print("ESP32 replied:", response)
        time.sleep(1)
        
except KeyboardInterrupt:
    print("\nClosing connection.")
    comms.close()

# "live, yes"
# "live, no"
# "dead, yes"
# "dead, no"

# "red, yes"
# "red, no"
# "green, yes"
# "green, no"
# "undefined, no"