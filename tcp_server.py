import socket
import time
import uselect


class PolledTcpServer:
    """Non-blocking TCP listener shared by the rotctld and web servers.

    Handles the socket lifecycle, the poller, and client tracking. Subclasses
    supply _read_client() to parse their own protocol."""

    # How often the stale client sweep runs.
    REAP_INTERVAL_MS = 1000

    def __init__(self, host, port, client_idle_ms, max_clients):
        self.host = host
        self.port = port
        self.client_idle_ms = client_idle_ms
        self.max_clients = max_clients
        self.server = None
        self.poller = uselect.poll()
        self.client_buffers = {}
        # Last time each client sent anything, used to close
        # stale connections.
        self.client_last_ms = {}
        self._last_reap_ms = time.ticks_ms()
        # Read by main.py so socket exhaustion reaches /api/log.
        self.last_accept_error = None

    def start(self):
        sock = socket.socket()
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind((self.host, self.port))
            sock.listen(2)
            sock.setblocking(False)
            self.poller.register(sock, uselect.POLLIN)
        except Exception:
            sock.close()
            raise
        self.server = sock

    def close(self):
        if self.server is None:
            return
        try:
            self.poller.unregister(self.server)
        except Exception:
            pass

        for client in list(self.client_buffers.keys()):
            self._close_client(client)

        self.server.close()
        self.server = None

    def restart(self):
        """Closes and reopens the listening socket. Called when Wi-Fi comes
        back, since a socket bound before the interface went down may no
        longer accept, or may hold an address DHCP has since changed."""
        self.close()
        self.start()

    def poll_once(self):
        if self.server is None:
            return

        now = time.ticks_ms()
        if time.ticks_diff(now, self._last_reap_ms) >= self.REAP_INTERVAL_MS:
            self._last_reap_ms = now
            self._reap_idle_clients(now)

        events = self.poller.poll(0)
        for sock_obj, event in events:
            if sock_obj is self.server:
                if event & uselect.POLLIN:
                    self._accept_client()
            else:
                if event & (uselect.POLLHUP | uselect.POLLERR):
                    self._close_client(sock_obj)
                    continue
                if event & uselect.POLLIN:
                    self._read_client(sock_obj)

    def _accept_client(self):
        # Makes room before accepting, so the accept itself cannot exhaust
        # the socket pool. Evicts the least recently active client.
        while len(self.client_buffers) >= self.max_clients:
            oldest = self._oldest_client()
            if oldest is None:
                break
            self._close_client(oldest)

        try:
            client, _ = self.server.accept()
            client.setblocking(False)
            self.poller.register(client, uselect.POLLIN | uselect.POLLHUP | uselect.POLLERR)
            self.client_buffers[client] = b""
            self.client_last_ms[client] = time.ticks_ms()
        except OSError as e:
            self.last_accept_error = repr(e)
            return

    def _close_client(self, client):
        try:
            self.poller.unregister(client)
        except Exception:
            pass
        try:
            client.close()
        except Exception:
            pass
        if client in self.client_buffers:
            del self.client_buffers[client]
        if client in self.client_last_ms:
            del self.client_last_ms[client]

    def _oldest_client(self):
        # Written out instead of min(key=..., default=...). Not every
        # MicroPython build supports that form.
        oldest = None
        oldest_ms = None
        for client, last in self.client_last_ms.items():
            if oldest_ms is None or time.ticks_diff(last, oldest_ms) < 0:
                oldest = client
                oldest_ms = last
        return oldest

    def _reap_idle_clients(self, now):
        for client in list(self.client_last_ms.keys()):
            last = self.client_last_ms.get(client)
            if last is None:
                continue
            if time.ticks_diff(now, last) >= self.client_idle_ms:
                self._close_client(client)

    def _read_client(self, client):
        raise NotImplementedError
