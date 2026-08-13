import time
import json

from tcp_server import PolledTcpServer

INDEX_HTML_FILE = "index.html"

STATUS_TEXT = {
    200: "OK",
    400: "Bad Request",
    403: "Forbidden",
    404: "Not Found",
    413: "Payload Too Large",
    500: "Internal Server Error",
}


class WebServer(PolledTcpServer):
    """Minimal non-blocking HTTP server exposing rotator control and LSM303
    calibration through a browser UI. No keep-alive: every response closes
    the connection."""

    MAX_REQUEST_BYTES = 8192

    def __init__(
        self,
        controller,
        sensor,
        store,
        default_tuning=None,
        wlan=None,
        log_buffer=None,
        host="0.0.0.0",
        port=80,
        client_idle_ms=15_000,
        max_clients=4,
    ):
        super().__init__(host, port, client_idle_ms, max_clients)
        self.controller = controller
        self.sensor = sensor
        self.store = store
        self.default_tuning = default_tuning if default_tuning is not None else {}
        self.wlan = wlan
        self.log_buffer = log_buffer if log_buffer is not None else []
        self._index_html, self._index_html_error = self._load_index_html()

    @staticmethod
    def _load_index_html(path=INDEX_HTML_FILE):
        try:
            with open(path) as f:
                return f.read().encode("utf-8"), None
        except OSError as e:
            return None, repr(e)

    def _read_client(self, client):
        try:
            data = client.recv(1024)
        except OSError:
            return

        self.client_last_ms[client] = time.ticks_ms()

        if not data:
            self._close_client(client)
            return

        buffer = self.client_buffers.get(client, b"") + data
        self.client_buffers[client] = buffer

        header_end = buffer.find(b"\r\n\r\n")
        if header_end == -1:
            if len(buffer) > self.MAX_REQUEST_BYTES:
                self._send_response(client, 400, "text/plain", b"Request too large")
                self._close_client(client)
            return

        header_text = buffer[:header_end].decode("utf-8", "ignore")
        lines = header_text.split("\r\n")
        parts = lines[0].split()
        if len(parts) < 2:
            self._send_response(client, 400, "text/plain", b"Bad request")
            self._close_client(client)
            return

        method, path = parts[0], parts[1]
        # Strips the query string instead of parsing it, so a client that
        # appends a cache-buster still matches its route.
        if "?" in path:
            path = path.partition("?")[0]

        # Only Content-Length is read from the request headers here.
        content_length = 0
        for line in lines[1:]:
            key, sep, value = line.partition(":")
            if not sep:
                continue
            if key.strip().lower() == "content-length":
                try:
                    content_length = int(value.strip())
                except ValueError:
                    content_length = 0

        body = buffer[header_end + 4:]
        if len(body) < content_length:
            if len(buffer) > self.MAX_REQUEST_BYTES:
                self._send_response(client, 413, "text/plain", b"Request too large")
                self._close_client(client)
            return

        body = body[:content_length]

        try:
            self._handle_request(client, method, path, body)
        except Exception as e:
            try:
                self._json_response(client, 500, {"error": str(e)})
            except Exception:
                pass
        finally:
            self._close_client(client)

    SEND_TIMEOUT_MS = 2000

    @classmethod
    def _send_all(cls, client, data):
        mv = memoryview(data)
        sent_total = 0
        deadline = time.ticks_add(time.ticks_ms(), cls.SEND_TIMEOUT_MS)
        while sent_total < len(mv):
            try:
                sent = client.send(mv[sent_total:])
            except OSError as e:
                if e.args[0] in (11, 35):  # EAGAIN or EWOULDBLOCK: buffer full, retry
                    if time.ticks_diff(deadline, time.ticks_ms()) <= 0:
                        raise
                    time.sleep_ms(1)
                    continue
                raise
            sent_total += sent

    def _send_response(self, client, status, content_type, body_bytes):
        header = (
            "HTTP/1.1 {} {}\r\n"
            "Content-Type: {}\r\n"
            "Content-Length: {}\r\n"
            "Connection: close\r\n"
            "\r\n"
        ).format(status, STATUS_TEXT.get(status, "OK"), content_type, len(body_bytes))
        try:
            self._send_all(client, header.encode("utf-8"))
            self._send_all(client, body_bytes)
        except OSError:
            pass

    def _json_response(self, client, status, obj):
        self._send_response(client, status, "application/json", json.dumps(obj).encode("utf-8"))

    @staticmethod
    def _parse_json(body):
        if not body:
            return {}
        try:
            return json.loads(body.decode("utf-8"))
        except Exception:
            return {}

    def _status_payload(self):
        az = None
        el = None
        if self.sensor.ready:
            try:
                az, el = self.controller.get_position()
            except Exception:
                pass
        return {
            "azimuth": az,
            "elevation": el,
            "target_azimuth": self.controller.target_az,
            "target_elevation": self.controller.target_el,
            "manual_direction": self.controller.manual_direction,
            "manual_speed": self.controller.manual_speed,
            "sensor_ready": self.sensor.ready,
            "last_error": self.controller.last_error,
            "wifi_connected": self.wlan.isconnected() if self.wlan is not None else None,
        }

    def _tuning_payload(self):
        payload = dict(self.controller.get_tuning())
        payload["defaults"] = dict(self.default_tuning)
        limits = getattr(self.controller, "TUNING_LIMITS", {})
        payload["limits"] = {name: list(bounds) for name, bounds in limits.items()}
        return payload

    def _handle_request(self, client, method, path, body):
        if method == "GET" and path == "/":
            if self._index_html is None:
                self._send_response(
                    client, 500, "text/plain",
                    "UI unavailable: {} ({})".format(
                        INDEX_HTML_FILE, self._index_html_error
                    ).encode("utf-8"),
                )
                return
            self._send_response(client, 200, "text/html; charset=utf-8", self._index_html)
            return

        if method == "GET" and path == "/api/status":
            self._json_response(client, 200, self._status_payload())
            return

        if method == "GET" and path == "/api/log":
            self._json_response(client, 200, {"log": list(self.log_buffer)})
            return

        if method == "POST" and path == "/api/move":
            payload = self._parse_json(body)
            direction = int(payload.get("direction"))
            speed = int(payload.get("speed", 60))
            self.controller.move(direction, speed)
            self._json_response(client, 200, {"ok": True})
            return

        if method == "POST" and path == "/api/stop":
            self.controller.stop()
            self._json_response(client, 200, {"ok": True})
            return

        if method == "POST" and path == "/api/goto":
            payload = self._parse_json(body)
            self.controller.set_target(float(payload.get("azimuth")), float(payload.get("elevation")))
            self._json_response(client, 200, {"ok": True})
            return

        if method == "POST" and path == "/api/park":
            self.controller.park()
            self._json_response(client, 200, {"ok": True})
            return

        if method == "POST" and path == "/api/reset":
            self.controller.reset()
            self._json_response(client, 200, {"ok": True})
            return

        if method == "GET" and path == "/api/tuning":
            self._json_response(client, 200, self._tuning_payload())
            return

        if method == "POST" and path == "/api/tuning":
            payload = self._parse_json(body)
            payload.pop("defaults", None)
            payload.pop("limits", None)
            try:
                self.controller.set_tuning(**payload)
            except ValueError as e:
                # Uses 400, not the blanket 500, since the UI shows this
                # message as-is to name the parameter and its bound.
                self._json_response(client, 400, {"error": str(e)})
                return
            self.store.save(self.store.TUNING_FILE, self.controller.get_tuning())
            self._json_response(client, 200, self._tuning_payload())
            return

        if method == "GET" and path == "/api/calibration":
            self._json_response(client, 200, self.sensor.get_calibration())
            return

        if method == "POST" and path == "/api/calibration":
            payload = self._parse_json(body)
            self.sensor.set_calibration(
                mag_declination_deg=payload.get("mag_declination_deg"),
                azimuth_offset_deg=payload.get("azimuth_offset_deg"),
                elevation_offset_deg=payload.get("elevation_offset_deg"),
                invert_azimuth=payload.get("invert_azimuth"),
                invert_elevation=payload.get("invert_elevation"),
                azimuth_forward_axis=payload.get("azimuth_forward_axis"),
                elevation_axis_pair=payload.get("elevation_axis_pair"),
                filter_strength=payload.get("filter_strength"),
                motion_threshold_deg=payload.get("motion_threshold_deg"),
            )
            self.store.save(self.store.CALIBRATION_FILE, self.sensor.get_calibration())
            self._json_response(client, 200, self.sensor.get_calibration())
            return

        if method == "POST" and path == "/api/calibration/mag/start":
            self.sensor.start_mag_calibration()
            self._json_response(client, 200, {"ok": True})
            return

        if method == "GET" and path == "/api/calibration/mag/status":
            self._json_response(client, 200, self.sensor.mag_calibration_progress())
            return

        if method == "POST" and path == "/api/calibration/mag/finish":
            result = self.sensor.finish_mag_calibration()
            self.store.save(self.store.CALIBRATION_FILE, self.sensor.get_calibration())
            self._json_response(client, 200, result)
            return

        if method == "POST" and path == "/api/calibration/mag/cancel":
            self.sensor.cancel_mag_calibration()
            self._json_response(client, 200, {"ok": True})
            return

        self._send_response(client, 404, "text/plain", b"Not found")
