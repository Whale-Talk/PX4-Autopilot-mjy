# -*- coding: utf-8 -*-
"""
ULog reading for the vibration tools.

Reader policy
-------------
* ``pyulog`` is the acceptance reader (already a repository dependency). It is
  called with ``message_name_filter_list`` limited to the topics this tooling
  needs, so unrelated topics are never materialised.
* ``auto`` uses pyulog when it is importable. If pyulog cannot decode the file,
  the load **fails** instead of silently falling back and reporting success.
  Use ``--reader native`` explicitly for a diagnostic read.
* ``native`` is a **diagnostic** fallback: it streams the file (no whole-file
  read), stores only the target topics, skips topics with unsupported (nested
  message) field types, and records that fact so the CLI can report limited
  evidence. It is not an acceptance-grade reader.

Structural facts verified against a real PX4 v1.16.1 log:
  * header ``b"ULog\\x01\\x125\\x01"``, uint64 start timestamp at offset 8;
  * messages are ``[uint16 payload_len][uint8 type][payload]``;
  * format strings declare generated ``_padding*`` fields and nested message
    types, data records contain no padding bytes (137/137 records checked);
  * ``ADD_LOGGED`` payload is ``[uint8 multi_id][uint16 msg_id][name]`` with **no
    NUL terminator** (162/162 records had ``payload_len == 3 + len(name)``).
"""

import os
import re
import struct

# ULog message types
MSG_FORMAT = 0x46        # 'F'
MSG_DATA = 0x44          # 'D'
MSG_INFO = 0x49          # 'I'
MSG_INFO_MULTI = 0x4D    # 'M'
MSG_PARAM = 0x50         # 'P'
MSG_PARAM_DEFAULT = 0x51  # 'Q'
MSG_ADD_LOGGED = 0x41    # 'A'
MSG_REMOVE_LOGGED = 0x52  # 'R'
MSG_SYNC = 0x53          # 'S'
MSG_DROPOUT = 0x4F       # 'O'
MSG_LOGGING = 0x4C       # 'L'
MSG_FLAG_BITS = 0x42     # 'B'

MSG_TYPE_NAMES = {
    MSG_FORMAT: "F format",
    MSG_DATA: "D data",
    MSG_INFO: "I info",
    MSG_INFO_MULTI: "M info_multi",
    MSG_PARAM: "P param",
    MSG_PARAM_DEFAULT: "Q default_param",
    MSG_ADD_LOGGED: "A add_logged",
    MSG_REMOVE_LOGGED: "R remove_logged",
    MSG_SYNC: "S sync",
    MSG_DROPOUT: "O dropout",
    MSG_LOGGING: "L logging",
    MSG_FLAG_BITS: "B flag_bits",
}

# base type -> (struct code, size, alignment)
BASE_TYPES = {
    "uint8_t": ("B", 1, 1),
    "int8_t": ("b", 1, 1),
    "bool": ("B", 1, 1),
    "char": ("c", 1, 1),
    "uint16_t": ("H", 2, 2),
    "int16_t": ("h", 2, 2),
    "uint32_t": ("I", 4, 4),
    "int32_t": ("i", 4, 4),
    "float": ("f", 4, 4),
    "uint64_t": ("Q", 8, 8),
    "int64_t": ("q", 8, 8),
    "double": ("d", 8, 8),
}

DEFAULT_STORE_TOPICS = ("sensor_accel_fifo", "sensor_gyro_fifo",
                        "logger_status", "vehicle_imu_status")

_ULOG_HEADER = b"ULog"
_ARRAY_KEY_RE = re.compile(r"^(?P<base>.+)\[(?P<index>\d+)\]$")


class UnsupportedFieldType(Exception):
    pass


class Field(object):
    __slots__ = ("name", "type_name", "count", "is_padding")

    def __init__(self, name, type_name, count):
        self.name = name
        self.type_name = type_name
        self.count = count
        self.is_padding = name.startswith("_padding")

    def __repr__(self):
        return "Field(%s:%s[%d])" % (self.name, self.type_name, self.count)


class Dataset(object):
    """One uORB subscription (topic + instance) with its field data."""

    __slots__ = ("name", "multi_id", "msg_id", "fields", "count")

    def __init__(self, name, multi_id, msg_id, fields):
        self.name = name
        self.multi_id = multi_id
        self.msg_id = msg_id
        self.fields = fields      # dict: field name -> list of values (arrays as list of rows)
        self.count = 0

    def has(self, field_name):
        return field_name in self.fields

    def get(self, field_name):
        return self.fields.get(field_name, [])

    def __repr__(self):
        return "Dataset(%s[%d], n=%d)" % (self.name, self.multi_id, self.count)


class UlogContent(object):
    def __init__(self, path):
        self.path = path
        self.reader = "unknown"
        self.size_bytes = os.path.getsize(path) if os.path.exists(path) else 0
        self.start_timestamp_us = None
        self.datasets = {}          # (name, multi_id) -> Dataset
        self.params = {}            # name -> value
        self.messages = []          # (timestamp_us, level, text)
        self.dropouts = []          # duration_ms values, in file order
        self.message_counts = {}    # type byte -> count
        self.unsupported_topics = []
        self.notes = []             # structural problems (truncation, bad key, ...)

    def dataset(self, name, multi_id=0):
        return self.datasets.get((name, multi_id))

    def datasets_named(self, name):
        return [d for (n, _mid), d in sorted(self.datasets.items()) if n == name]

    def topic_names(self):
        return sorted(set(name for name, _mid in self.datasets.keys()))

    def counts_by_name(self):
        out = {}
        for key, value in self.message_counts.items():
            out[MSG_TYPE_NAMES.get(key, hex(key))] = value
        return out


def _parse_format(text):
    """'name:uint64_t timestamp;uint8_t[32] x;' -> (name, [Field, ...])"""
    if ":" not in text:
        return None, None
    name, body = text.split(":", 1)
    name = name.strip()
    fields = []
    for raw in body.split(";"):
        raw = raw.strip()
        if not raw:
            continue
        bits = raw.split()
        if len(bits) < 2:
            continue
        type_name, field_name = bits[0], bits[1]
        count = 1
        if "[" in type_name:
            type_name, rest = type_name.split("[", 1)
            count = int(rest.rstrip("]"))
        fields.append(Field(field_name, type_name, count))
    if not name or not fields:
        return None, None
    return name, fields


def format_has_unsupported_fields(fields):
    return any((not field.is_padding) and field.type_name not in BASE_TYPES for field in fields)


def _decode_value(field, raw):
    type_name, count = field.type_name, field.count
    if type_name not in BASE_TYPES:
        raise UnsupportedFieldType(type_name)
    code, size, _align = BASE_TYPES[type_name]
    if type_name == "char":
        text = raw[:size * count]
        return text.split(b"\x00", 1)[0].decode("utf-8", "replace")
    if count == 1:
        return struct.unpack_from("<" + code, raw, 0)[0]
    return list(struct.unpack_from("<%d%s" % (count, code), raw, 0))


def iter_messages(path):
    """Stream a ULog file: yields (msg_type, payload) per message.

    The file is read message by message through a buffered handle; the whole file
    is never loaded into memory.
    """
    with open(path, "rb") as handle:
        header = handle.read(16)
        if len(header) < 16 or header[:4] != _ULOG_HEADER:
            raise ValueError("not a ULog file (bad header): %r" % header[:8])
        yield ("start", struct.unpack_from("<Q", header, 8)[0])

        while True:
            raw_header = handle.read(3)
            if not raw_header:
                return
            if len(raw_header) < 3:
                yield ("truncated-header", raw_header)
                return
            payload_len, msg_type = struct.unpack("<HB", raw_header)
            payload = handle.read(payload_len) if payload_len else b""
            if len(payload) < payload_len:
                yield ("truncated-payload", (msg_type, payload_len, len(payload)))
                return
            yield (msg_type, payload)


def scan_structure(path, decode=False, store_topics=DEFAULT_STORE_TOPICS):
    """Stream the file once.

    With ``decode=True`` data records of ``store_topics`` are decoded into
    datasets; every other topic is only inspected for unsupported field types
    (recorded in ``content.unsupported_topics``), which keeps memory bounded.
    """
    content = UlogContent(path)
    formats = {}
    subscriptions = {}     # msg_id -> (name, multi_id, [Field, ...])

    for entry in iter_messages(path):
        if entry[0] == "start":
            content.start_timestamp_us = entry[1]
            continue
        if entry[0] == "truncated-header":
            content.notes.append("truncated message header at end of file")
            break
        if entry[0] == "truncated-payload":
            msg_type, expected, got = entry[1]
            content.notes.append("truncated payload for message type %s (expected %d, got %d)"
                                 % (chr(msg_type) if 32 <= msg_type < 127 else hex(msg_type), expected, got))
            break

        msg_type, payload = entry
        content.message_counts[msg_type] = content.message_counts.get(msg_type, 0) + 1

        try:
            if msg_type == MSG_FORMAT:
                name, fields = _parse_format(payload.decode("utf-8", "replace").rstrip("\x00"))
                if name:
                    formats[name] = fields

            elif msg_type == MSG_ADD_LOGGED:
                if len(payload) < 3:
                    content.notes.append("ADD_LOGGED message too short (%d bytes)" % len(payload))
                    continue
                multi_id = payload[0]
                msg_id = struct.unpack_from("<H", payload, 1)[0]
                # no NUL terminator: the payload length defines the topic name
                name = payload[3:].split(b"\x00", 1)[0].decode("utf-8", "replace")
                fields = formats.get(name)
                if fields is not None:
                    subscriptions[msg_id] = (name, multi_id, fields)
                    if format_has_unsupported_fields(fields) and name not in content.unsupported_topics:
                        content.unsupported_topics.append(name)

            elif msg_type == MSG_DATA and decode:
                if len(payload) < 2:
                    content.notes.append("DATA message too short (%d bytes)" % len(payload))
                    continue
                msg_id = struct.unpack_from("<H", payload, 0)[0]
                sub = subscriptions.get(msg_id)
                if sub is None:
                    continue
                name, multi_id, fields = sub
                if store_topics and name not in store_topics:
                    continue
                if format_has_unsupported_fields(fields):
                    continue
                key = (name, multi_id)
                ds = content.datasets.get(key)
                if ds is None:
                    ds = Dataset(name, multi_id, msg_id,
                                 {f.name: [] for f in fields if not f.is_padding})
                    content.datasets[key] = ds
                cursor = 2
                for field in fields:
                    if field.is_padding:
                        continue
                    ds.fields[field.name].append(_decode_value(field, payload[cursor:]))
                    cursor += BASE_TYPES[field.type_name][1] * field.count
                ds.count += 1

            elif msg_type in (MSG_INFO, MSG_PARAM, MSG_PARAM_DEFAULT):
                if not payload:
                    continue
                key_len = payload[0]
                if 1 + key_len > len(payload):
                    content.notes.append("key-length message truncated (key_len %d, payload %d)"
                                         % (key_len, len(payload)))
                    continue
                key = payload[1:1 + key_len].decode("utf-8", "replace")
                _store_key_value(content, msg_type, key, payload[1 + key_len:])

            elif msg_type == MSG_INFO_MULTI:
                if len(payload) < 2:
                    continue
                key_len = payload[1]
                if 2 + key_len > len(payload):
                    content.notes.append("multi-info message truncated (key_len %d, payload %d)"
                                         % (key_len, len(payload)))
                    continue
                key = payload[2:2 + key_len].decode("utf-8", "replace")
                _store_key_value(content, msg_type, key, payload[2 + key_len:])

            elif msg_type == MSG_DROPOUT:
                if len(payload) >= 2:
                    content.dropouts.append(struct.unpack_from("<H", payload, 0)[0])

            elif msg_type == MSG_LOGGING:
                if len(payload) >= 9:
                    content.messages.append((struct.unpack_from("<Q", payload, 1)[0], payload[0],
                                             payload[9:].split(b"\x00", 1)[0].decode("utf-8", "replace")))

        except Exception as exc:  # one bad record must not hide the rest
            if len(content.notes) < 50:
                content.notes.append("record (type %s) not decoded: %s"
                                     % (chr(msg_type) if 32 <= msg_type < 127 else hex(msg_type), exc))

    return content, subscriptions, formats


def _store_key_value(content, msg_type, key, raw):
    bits = key.split(" ", 1)
    type_name = bits[0]
    name = bits[1] if len(bits) > 1 else ""
    prefix = "_info_" if msg_type in (MSG_INFO, MSG_INFO_MULTI) else ""
    if type_name.startswith("char["):
        length = int(type_name[5:-1])
        content.params[prefix + name] = raw[:length].split(b"\x00", 1)[0].decode("utf-8", "replace")
    else:
        code_size = BASE_TYPES.get(type_name)
        if code_size is None:
            return
        code, size, _align = code_size
        if len(raw) >= size:
            content.params[prefix + name] = struct.unpack_from("<" + code, raw, 0)[0]


def normalize_pyulog_fields(fields):
    """Rebuild arrays that pyulog flattened into ``name[i]`` keys.

    pyulog exposes a fixed array field as ``x[0]`` .. ``x[31]``; this turns it back
    into ``x = [[...32 values...], ...]`` (one row per record) so that the analysis
    code sees the same shape as the native reader. Also applies to
    ``vehicle_imu_status.accel_clipping`` etc.
    """
    scalars = {}
    arrays = {}
    for key, values in fields.items():
        match = _ARRAY_KEY_RE.match(key)
        if match:
            arrays.setdefault(match.group("base"), {})[int(match.group("index"))] = list(values)
        else:
            scalars[key] = list(values)

    for base, by_index in arrays.items():
        indices = sorted(by_index)
        if not indices:
            continue
        rows = len(by_index[indices[0]])
        scalars[base] = [[by_index[i][row] for i in indices] for row in range(rows)]

    return scalars


def pyulog_available():
    try:
        import pyulog  # noqa: F401
        return True
    except Exception:
        return False


def _load_with_pyulog(path, topics):
    import pyulog

    content, _subs, _fmts = scan_structure(path, decode=False)
    content.reader = "pyulog"

    try:
        ulog = pyulog.ULog(path, message_name_filter_list=list(topics))
    except TypeError:
        # very old pyulog without the filter argument: fall back to a full load
        ulog = pyulog.ULog(path)
        content.notes.append("pyulog without message_name_filter_list: all topics were loaded")

    for data in ulog.data_list:
        fields = normalize_pyulog_fields(data.data)
        dataset = Dataset(data.name, data.multi_id, getattr(data, "msg_id", -1), fields)
        for key in ("timestamp", "timestamp_sample"):
            if key in fields:
                dataset.count = len(fields[key])
                break
        else:
            dataset.count = len(next(iter(fields.values()))) if fields else 0
        content.datasets[(data.name, data.multi_id)] = dataset

    for key, value in (ulog.initial_parameters or {}).items():
        content.params.setdefault(key, value)

    for message in getattr(ulog, "logged_messages", []) or []:
        content.messages.append((getattr(message, "timestamp", 0),
                                 getattr(message, "log_level", -1),
                                 getattr(message, "message", "")))
    return content


def load_ulog(path, reader="auto", topics=DEFAULT_STORE_TOPICS):
    """Load a ULog file.

    reader: "auto" (pyulog when installed, otherwise the diagnostic native reader),
    "pyulog" (must succeed) or "native" (diagnostic reader).
    The backend actually used is always recorded in ``content.reader``.
    """
    if reader not in ("auto", "pyulog", "native"):
        raise ValueError("unknown reader %r" % reader)

    topics = tuple(topics) if topics else DEFAULT_STORE_TOPICS

    if reader in ("auto", "pyulog"):
        if pyulog_available():
            try:
                return _load_with_pyulog(path, topics)
            except Exception as exc:
                if reader == "pyulog":
                    raise
                raise RuntimeError(
                    "pyulog could not decode %s (%s). This is not silently downgraded: "
                    "re-run with --reader native for a diagnostic read, which is limited and "
                    "not an acceptance-grade result." % (path, exc))
        if reader == "pyulog":
            raise RuntimeError("pyulog is not installed in this interpreter")

    content, _subs, _fmts = scan_structure(path, decode=True, store_topics=topics)
    content.reader = "native (diagnostic%s)" % ("" if pyulog_available() else "; pyulog not installed")
    return content
