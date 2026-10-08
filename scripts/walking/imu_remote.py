"""Cote portable : recoit tangage et roulis du torse envoyes par scripts/imu_sender.py (sur le Pi)."""
import socket
import struct
import threading
import time

PACKET = struct.Struct('<7d')


class ImuClient:
    def __init__(self, host, port=9001):
        self._addr = (host, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(0.5)
        self._lock = threading.Lock()
        self._last = None          # (tangage, roulis, vit. tangage, vit. roulis)
        self._last_rx = 0.0
        self.count = 0
        self.i2c_errors = 0
        self.yaw = 0.0             # cap cumule (deg, + = vers la gauche)
        self._running = True
        threading.Thread(target=self._hello_loop, daemon=True).start()
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def _hello_loop(self):
        while self._running:
            try:
                self._sock.sendto(b'imu?', self._addr)
            except OSError:
                pass
            time.sleep(1.0)

    def _recv_loop(self):
        while self._running:
            try:
                data, _ = self._sock.recvfrom(256)
            except (socket.timeout, OSError):
                continue
            if len(data) != PACKET.size:
                continue
            _, pitch, roll, pitch_rate, roll_rate, errors, yaw = PACKET.unpack(data)
            with self._lock:
                self._last = (pitch, roll, pitch_rate, roll_rate)
                self._last_rx = time.perf_counter()
                self.count += 1
                self.i2c_errors = int(errors)
                self.yaw = yaw

    def get(self, max_age=0.2):
        """Renvoie (tangage, roulis, vit. tangage, vit. roulis, age en s), ou None si rien de recent."""
        with self._lock:
            if self._last is None:
                return None
            age = time.perf_counter() - self._last_rx
            if age > max_age:
                return None
            return (*self._last, age)

    def close(self):
        self._running = False
