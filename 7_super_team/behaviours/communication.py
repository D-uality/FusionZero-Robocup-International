import socket
import time
from hardware.robot import *

class Communications:
    def __init__(self):
        self.IP = "192.168.4.1"
        self.PORT = 1234
        self.send_message = "undefined, no"
        self.received_message = "undefined, no"
        self.RECONNECT_INTERVAL = 3
        self.last_reconnect_attempt = time.monotonic()
        self.DummySocket = self._create_dummy_socket().__class__
        self._connect()
    
    def _connect(self):
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect((self.IP, self.PORT))
            sock.settimeout(None)
            self.sock = sock
            print("Connected to ESP32.")
            oled_display.text("ESP32 ✓", 70, 45)
            time.sleep(1)
        except (socket.timeout, ConnectionRefusedError, OSError) as e:
            print(f"Initial connection failed: {e}")
            self.sock = self._create_dummy_socket()
    
    def _create_dummy_socket(self):
        class DummySocket:
            def sendall(self, data):
                pass
            def recv(self, size):
                return b"undefined, no"
            def close(self):
                pass
            def getsockopt(self, *args):
                raise socket.error("Dummy socket")
            def setblocking(self, flag):
                pass
        return DummySocket()
    
    def _is_connection_alive(self) -> bool:
        if isinstance(self.sock, self.DummySocket):
            return False
        
        try:
            # Check socket error state
            error = self.sock.getsockopt(socket.SOL_SOCKET, socket.SO_ERROR)
            if error != 0:
                return False
            
            # Quick non-blocking check
            self.sock.setblocking(False)
            try:
                self.sock.recv(1, socket.MSG_PEEK)
            except socket.error as e:
                if e.errno not in (socket.EAGAIN, socket.EWOULDBLOCK):
                    return False
            finally:
                self.sock.setblocking(True)
            
            return True
        except (socket.error, OSError):
            return False
    
    def _maybe_reconnect(self):
        if not self._is_connection_alive():
            now = time.monotonic()
            if now - self.last_reconnect_attempt >= self.RECONNECT_INTERVAL:
                print("Attempting to reconnect to ESP32...")
                self.last_reconnect_attempt = now
                self._connect()
    
    def send(self) -> None:
        self._maybe_reconnect()
        formatted_data = f"{self.send_message}\n"
        try:
            self.sock.sendall(formatted_data.encode())
        except (socket.error, OSError) as e:
            print(f"Send error: {e}")
            self.sock = self._create_dummy_socket()
    
    def receive(self) -> str:
        self._maybe_reconnect()
        try:
            data = self.sock.recv(1024)
            if not data:  # Connection closed by peer
                raise socket.error("Connection closed by peer")
            self.received_message = data.decode().strip()
        except (socket.error, OSError) as e:
            print(f"Receive error: {e}")
            self.received_message = f"Error: {e}"
            self.sock = self._create_dummy_socket()
            
        print(self.received_message)
        return self.received_message
    
    def close(self):
        try:
            if not isinstance(self.sock, self.DummySocket):
                self.sock.close()
        except Exception:
            pass

# Usage
comms = Communications()

if __name__ == "__main__":
    while True:
        comms.send()
        comms.receive()
        print(f"{time.perf_counter():.2f} RECEIVED: {comms.received_message}")
        time.sleep(1)