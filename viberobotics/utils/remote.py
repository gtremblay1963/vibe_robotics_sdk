import socket
import struct
import json
import threading
from collections import deque
import numpy as np


class NumpySocket:
    """
    One-way NumPy array transport over TCP.

    Receiver:
      - A background thread reads arrays from the socket and appends to a buffer.
      - recv(min_ready=n) returns ONE array, but blocks until buffer has >= n arrays.
        This ensures the consumer stays ahead of network jitter.

    Sender:
      - send(arr) writes header + raw bytes.
    """

    def __init__(self, host="127.0.0.1", port=9000, is_sender=False, buffer_max=512):
        self.host = host
        self.port = port
        self.is_sender = is_sender

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        self._conn = None  # receiver accepted connection
        self._closed = False

        # Receiver-side buffer + condition variable
        self._buffer = deque(maxlen=buffer_max)
        self._cv = threading.Condition()
        self._reader_thread = None
        self._reader_exc = None

        if self.is_sender:
            self._sock.connect((host, port))
        else:
            # Port reutilisable tout de suite apres l'arret d'un ancien recepteur (sinon « Address already in use »
            # pendant ~60 s, le temps que le systeme libere l'ancienne connexion)
            self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self._sock.bind((host, port))
            self._sock.listen(1)
            print(f"[Receiver] Listening on {host}:{port} ...")
            self._conn, addr = self._sock.accept()
            self._conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            print(f"[Receiver] Connected from {addr}")

            self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
            self._reader_thread.start()

    # ----------------- low-level IO -----------------
    def _sendall(self, b: bytes):
        self._sock.sendall(b)

    def _recvall(self, n: int) -> bytes:
        assert self._conn is not None
        data = b""
        while len(data) < n:
            chunk = self._conn.recv(n - len(data))
            if not chunk:
                raise ConnectionError("Socket closed by peer")
            data += chunk
        return data

    # ----------------- framing -----------------
    @staticmethod
    def _pack_header(arr: np.ndarray) -> bytes:
        header = {
            "shape": arr.shape,
            "dtype": str(arr.dtype),
            "nbytes": int(arr.nbytes),
        }
        hb = json.dumps(header).encode("utf-8")
        return struct.pack("!I", len(hb)) + hb

    def _read_one_array_from_socket(self) -> np.ndarray:
        header_len = struct.unpack("!I", self._recvall(4))[0]
        header = json.loads(self._recvall(header_len).decode("utf-8"))

        shape = tuple(header["shape"])
        dtype = np.dtype(header["dtype"])
        nbytes = int(header["nbytes"])

        raw = self._recvall(nbytes)
        return np.frombuffer(raw, dtype=dtype).reshape(shape)

    # ----------------- receiver background loop -----------------
    def _reader_loop(self):
        try:
            while not self._closed:
                arr = self._read_one_array_from_socket()
                with self._cv:
                    self._buffer.append(arr)   # ring buffer; drops oldest if full
                    self._cv.notify_all()
        except Exception as e:
            with self._cv:
                self._reader_exc = e
                self._cv.notify_all()

    # ----------------- public API -----------------
    def send(self, arr: np.ndarray):
        if not self.is_sender:
            raise RuntimeError("send() called on receiver")

        arr = np.ascontiguousarray(arr)
        header = self._pack_header(arr)
        self._sendall(header)
        self._sendall(arr.tobytes())

    def recv(self, *, min_ready: int = 1, timeout: float | None = None) -> np.ndarray:
        """
        Receiver-only: returns ONE array, but blocks until buffer has >= min_ready.
        Typical usage: recv(min_ready=10) to keep at least 10 queued before consuming.
        """
        if self.is_sender:
            raise RuntimeError("recv() called on sender")
        if min_ready < 1:
            raise ValueError("min_ready must be >= 1")

        with self._cv:
            ok = self._cv.wait_for(
                lambda: (len(self._buffer) >= min_ready) or (self._reader_exc is not None) or self._closed,
                timeout=timeout,
            )
            if not ok:
                raise TimeoutError(f"Timed out waiting for buffer >= {min_ready} (have {len(self._buffer)})")
            if self._reader_exc is not None:
                raise RuntimeError(f"Receiver reader thread died: {self._reader_exc}") from self._reader_exc
            if self._closed:
                raise ConnectionError("Receiver closed")

            # Now we *only pop one*, but we ensured a cushion exists.
            return self._buffer.popleft()

    def buffer_size(self) -> int:
        if self.is_sender:
            return 0
        with self._cv:
            return len(self._buffer)

    def close(self):
        self._closed = True
        with self._cv:
            self._cv.notify_all()
        try:
            if self._conn is not None:
                self._conn.close()
        except Exception:
            pass
        try:
            self._sock.close()
        except Exception:
            pass
