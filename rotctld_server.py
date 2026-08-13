import time

from tcp_server import PolledTcpServer

RPRT_OK = 0
RPRT_INVALID_PARAM = -1
RPRT_UNSUPPORTED = -4
RPRT_INTERNAL = -9


class RotctldServer(PolledTcpServer):
    """Minimal rotctld-compatible TCP server for a dual-axis rotator."""

    LONG_TO_SHORT = {
        "set_pos": "P",
        "get_pos": "p",
        "move": "M",
        "stop": "S",
        "park": "K",
        "reset": "R",
        "get_info": "_",
        "dump_state": "dump_state",
        "dump_caps": "1",
    }

    LOCATOR_COMMANDS = {"L", "l", "D", "d", "E", "e", "B", "A", "a"}

    def __init__(
        self,
        controller,
        host="0.0.0.0",
        port=4533,
        client_idle_ms=300_000,
        max_clients=4,
    ):
        super().__init__(host, port, client_idle_ms, max_clients)
        self.controller = controller

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

        while b"\n" in buffer:
            line, buffer = buffer.split(b"\n", 1)
            line = line.strip(b"\r")
            response = self._handle_line(line.decode("utf-8", "ignore"))
            if response:
                try:
                    client.send(response.encode("utf-8"))
                except OSError:
                    self._close_client(client)
                    return

        self.client_buffers[client] = buffer

    @staticmethod
    def _parse_erp(line):
        if not line:
            return None, ""
        first = line[0]
        is_alnum = first.isalpha() or first.isdigit()
        if not is_alnum and first not in "\\?_#":
            if first == "+":
                return "\n", line[1:].strip()
            return first, line[1:].strip()
        return None, line.strip()

    @staticmethod
    def _parse_command(cmd_line):
        if not cmd_line:
            return None, []

        parts = cmd_line.split()
        head = parts[0]
        args = parts[1:]

        if head.startswith("\\"):
            return head[1:], args

        # Long commands are accepted with or without the leading backslash.
        return head, args

    @staticmethod
    def _format_float(value):
        return "{:.6f}".format(float(value))

    def _reply_default(self, is_get, rprt, value_lines):
        if rprt != RPRT_OK:
            return "RPRT {}\n".format(rprt)
        if is_get:
            if not value_lines:
                return "\n"
            return "\n".join(value_lines) + "\n"
        return "RPRT 0\n"

    def _reply_extended(self, separator, long_name, args, rprt, kv_pairs):
        records = []

        if args:
            records.append("{}: {}".format(long_name, " ".join(args)))
        else:
            records.append("{}:".format(long_name))

        for key, value in kv_pairs:
            records.append("{}: {}".format(key, value))

        records.append("RPRT {}".format(rprt))

        if separator == "\n":
            return "\n".join(records) + "\n"

        return separator.join(records)

    def _build_dump_state(self):
        az, el = self.controller.get_position()
        lines = [
            "0",  # protocol version placeholder
            "450",  # min az
            "-450",  # max az
            "{}".format(self.controller.min_elevation_deg),
            "{}".format(self.controller.max_elevation_deg),
            "0",  # min speed
            "100",  # max speed
            self._format_float(az),
            self._format_float(el),
        ]
        return lines

    def _build_dump_caps(self):
        lines = [
            "Model name: Rot32",
            "Mfg name: Rot32",
            "Backend version: 0.1",
            "Has set_pos: Y",
            "Has get_pos: Y",
            "Has move: Y",
            "Has stop: Y",
            "Has park: Y",
            "Has reset: Y",
            "Has set_conf: N",
            "Has get_info: Y",
            "Has locator_ops: N",
        ]
        return lines

    def _execute(self, command_name, args):
        short_cmd = command_name
        if command_name in self.LONG_TO_SHORT:
            short_cmd = self.LONG_TO_SHORT[command_name]

        if short_cmd in self.LOCATOR_COMMANDS:
            return "unsupported", RPRT_UNSUPPORTED, [], []

        try:
            if short_cmd == "P":
                if len(args) < 2:
                    return "set_pos", RPRT_INVALID_PARAM, [], []
                az = float(args[0])
                el = float(args[1])
                self.controller.set_target(az, el)
                return "set_pos", RPRT_OK, [], []

            if short_cmd == "p":
                az, el = self.controller.get_position()
                value_lines = [self._format_float(az), self._format_float(el)]
                kv = [("Azimuth", value_lines[0]), ("Elevation", value_lines[1])]
                return "get_pos", RPRT_OK, value_lines, kv

            if short_cmd == "M":
                if len(args) < 2:
                    return "move", RPRT_INVALID_PARAM, [], []
                direction = int(args[0])
                speed = int(args[1])
                if speed < 0:
                    speed = 0
                self.controller.move(direction, speed)
                return "move", RPRT_OK, [], []

            if short_cmd == "S":
                self.controller.stop()
                return "stop", RPRT_OK, [], []

            if short_cmd == "K":
                self.controller.park()
                return "park", RPRT_OK, [], []

            if short_cmd == "R":
                if len(args) < 1:
                    return "reset", RPRT_INVALID_PARAM, [], []
                reset_type = int(args[0])
                if reset_type != 1:
                    return "reset", RPRT_INVALID_PARAM, [], []
                self.controller.reset()
                return "reset", RPRT_OK, [], []

            if short_cmd == "_":
                info = "Rot32"
                return "get_info", RPRT_OK, [info], [("Info", info)]

            if short_cmd == "dump_state":
                values = self._build_dump_state()
                kv = [("State{}".format(i + 1), values[i]) for i in range(len(values))]
                return "dump_state", RPRT_OK, values, kv

            if short_cmd == "1":
                values = self._build_dump_caps()
                kv = [("Cap{}".format(i + 1), values[i]) for i in range(len(values))]
                return "dump_caps", RPRT_OK, values, kv

            return "unknown", RPRT_UNSUPPORTED, [], []

        except Exception:
            return "internal", RPRT_INTERNAL, [], []

    def _handle_line(self, line):
        separator, cmd_line = self._parse_erp(line)
        command_name, args = self._parse_command(cmd_line)

        if command_name is None:
            return ""

        long_name, rprt, value_lines, kv_pairs = self._execute(command_name, args)

        if separator is not None:
            return self._reply_extended(separator, long_name, args, rprt, kv_pairs)

        is_get = long_name in ("get_pos", "get_info", "dump_state", "dump_caps")
        return self._reply_default(is_get, rprt, value_lines)
