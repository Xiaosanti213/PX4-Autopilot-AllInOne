"""
ulog_parser.py - PX4 ULog binary format parser

Pure Python implementation, no external dependencies.
Reference: https://docs.px4.io/main/en/log/ulog_file_format.html

Supports:
- Full ULog header, definition, and data sections
- FORMAT, ADD_LOGGED_MSG, INFO, PARAMETER, DATA messages
- All standard scalar and array field types
- Large file handling via mmap

Usage:
    from ulog_parser import ULogParser
    ulog = ULogParser("flight.ulg").parse()
    attitude_data = ulog.topics.get('vehicle_attitude', [])
"""

import struct
import mmap
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any
from pathlib import Path


# ============================================================
# Data Classes
# ============================================================

@dataclass
class ULogHeader:
    """ULog file header (first 16 bytes)"""
    magic: bytes          # b'ULog'
    version: int          # file format version
    timestamp: int        # start timestamp in microseconds


@dataclass
class FieldDef:
    """Single field definition within a FORMAT message"""
    type_str: str
    name: str
    array_size: int = 0
    is_padding: bool = False

    @property
    def size(self) -> int:
        type_sizes = {
            'int8': 1, 'uint8': 1, 'int16': 2, 'uint16': 2,
            'int32': 4, 'uint32': 4, 'int64': 8, 'uint64': 8,
            'float': 4, 'double': 8, 'bool': 1, 'char': 1,
        }
        base = type_sizes.get(self.type_str, 0)
        return base * (self.array_size if self.array_size > 0 else 1)


@dataclass
class TopicFormat:
    """Topic message format (from FORMAT + ADD_LOGGED_MSG)"""
    msg_id: int
    name: str
    fields: List[FieldDef]

    @property
    def msg_size(self) -> int:
        """Full struct size including padding"""
        return sum(f.size for f in self.fields)

    @property
    def data_size(self) -> int:
        """Data size excluding trailing padding fields"""
        return sum(f.size for f in self.fields if not f.is_padding)

    @property
    def non_padding_fields(self) -> List[FieldDef]:
        """Fields excluding trailing padding"""
        return [f for f in self.fields if not f.is_padding]


@dataclass
class TopicData:
    """Single topic data entry (one DATA message)"""
    timestamp: int               # microseconds
    topic_name: str
    values: Dict[str, Any]       # field_name -> decoded value


@dataclass
class ULogFile:
    """Fully parsed ULog file"""
    header: ULogHeader
    topics: Dict[str, List[TopicData]]    # topic_name -> list of entries
    formats: Dict[int, TopicFormat]       # msg_id -> format
    info: Dict[str, str]                  # INFO key-value pairs
    parameters: Dict[str, Any]            # PARAMETER name -> value

    @property
    def start_timestamp_us(self) -> int:
        """Earliest timestamp across all topics"""
        ts = []
        for entries in self.topics.values():
            if entries:
                ts.append(entries[0].timestamp)
        return min(ts) if ts else 0

    @property
    def end_timestamp_us(self) -> int:
        """Latest timestamp across all topics"""
        ts = []
        for entries in self.topics.values():
            if entries:
                ts.append(entries[-1].timestamp)
        return max(ts) if ts else 0

    @property
    def duration_s(self) -> float:
        """Flight duration in seconds"""
        return (self.end_timestamp_us - self.start_timestamp_us) / 1_000_000.0

    def get_topic(self, name: str) -> List[TopicData]:
        """Get data entries for a topic, empty list if not found"""
        return self.topics.get(name, [])

    def has_topic(self, name: str) -> bool:
        """Check if a topic exists in the log"""
        return name in self.topics and len(self.topics[name]) > 0

    def get_param(self, name: str, default: Any = None) -> Any:
        """Get a parameter value"""
        return self.parameters.get(name, default)


# ============================================================
# Parser
# ============================================================

class ULogParser:
    """Parse ULog files - pure Python, zero dependencies.

    For large files (>100MB), uses mmap to avoid loading entire file into memory.
    """

    TYPE_SIZES = {
        'int8': 1, 'uint8': 1, 'int16': 2, 'uint16': 2,
        'int32': 4, 'uint32': 4, 'int64': 8, 'uint64': 8,
        'float': 4, 'double': 8, 'bool': 1, 'char': 1,
    }

    def __init__(self, path: str):
        self.path = Path(path)
        self._raw: Optional[bytes] = None
        self._mm: Optional[mmap.mmap] = None
        self._file_handle = None

    def _get_raw(self) -> bytes:
        """Load file content - uses mmap for large files"""
        if self._raw is not None:
            return self._raw
        file_size = self.path.stat().st_size
        if file_size > 100 * 1024 * 1024:  # >100MB: use mmap
            self._file_handle = open(self.path, 'rb')
            self._mm = mmap.mmap(self._file_handle.fileno(), 0, access=mmap.ACCESS_READ)
            self._raw = self._mm
        else:
            self._raw = self.path.read_bytes()
        return self._raw

    def parse(self) -> ULogFile:
        """Parse entire ULog file and return structured data."""
        raw = self._get_raw()
        if len(raw) < 16:
            raise ValueError("File too small to be a valid ULog")

        # Phase 1: Parse header
        header = self._parse_header(raw)

        # Phase 2: Scan definition section (FORMAT, ADD_LOGGED_MSG, INFO, PARAMETER)
        formats_by_name: Dict[str, TopicFormat] = {}
        subscriptions: Dict[int, str] = {}   # msg_id -> topic_name
        info: Dict[str, str] = {}
        parameters: Dict[str, Any] = {}

        scan_pos = 16
        while scan_pos < len(raw):
            if scan_pos + 3 > len(raw):
                break
            size = int.from_bytes(raw[scan_pos:scan_pos + 2], 'little')
            msg_type = raw[scan_pos + 2]
            scan_pos += 3

            if size == 0 or scan_pos + size > len(raw):
                break

            msg_data = raw[scan_pos:scan_pos + size]
            scan_pos += size

            if msg_type == ord('F'):    # FORMAT
                fmt = self._parse_format_msg(msg_data)
                if fmt:
                    formats_by_name[fmt.name] = fmt
            elif msg_type == ord('A'):  # ADD_LOGGED_MSG
                self._parse_subscription(msg_data, subscriptions)
            elif msg_type == ord('I'):  # INFO
                self._parse_info_msg(msg_data, info)
            elif msg_type == ord('P'):  # PARAMETER
                self._parse_parameter_msg(msg_data, parameters)
            elif msg_type == ord('D'):  # DATA - definition section ends here
                break

        # Build msg_id -> format mapping
        formats_by_id: Dict[int, TopicFormat] = {}
        for msg_id, name in subscriptions.items():
            if name in formats_by_name:
                fmt = formats_by_name[name]
                fmt.msg_id = msg_id
                formats_by_id[msg_id] = fmt

        # Phase 3: Parse data section (all DATA messages)
        topics: Dict[str, List[TopicData]] = {}
        scan_pos = 16
        while scan_pos < len(raw):
            if scan_pos + 3 > len(raw):
                break
            size = int.from_bytes(raw[scan_pos:scan_pos + 2], 'little')
            msg_type = raw[scan_pos + 2]
            scan_pos += 3

            if size == 0 or scan_pos + size > len(raw):
                break

            msg_data = raw[scan_pos:scan_pos + size]
            scan_pos += size

            if msg_type == ord('D'):
                self._parse_data_msg(msg_data, formats_by_id, topics)

        self._cleanup()

        return ULogFile(
            header=header,
            topics=topics,
            formats=formats_by_id,
            info=info,
            parameters=parameters,
        )

    def _cleanup(self):
        """Release file resources"""
        if self._mm:
            self._mm.close()
        if self._file_handle:
            self._file_handle.close()
        self._mm = None
        self._file_handle = None

    def _parse_header(self, raw: bytes) -> ULogHeader:
        """Parse 16-byte file header"""
        magic = raw[0:4]
        if magic != b'ULog':
            raise ValueError(f"Invalid ULog magic bytes: {magic}")
        version = raw[7]
        timestamp = int.from_bytes(raw[8:16], 'little')
        return ULogHeader(magic=magic, version=version, timestamp=timestamp)

    def _parse_format_msg(self, data: bytes) -> Optional[TopicFormat]:
        """Parse FORMAT message: 'topic_name:field_type field_name;...'"""
        try:
            text = data.decode('utf-8', errors='replace')
            if ':' not in text:
                return None
            name, fields_str = text.split(':', 1)
            fields = self._parse_field_defs(fields_str)
            # Remove trailing padding fields
            while fields and fields[-1].is_padding:
                fields.pop()
            return TopicFormat(msg_id=0, name=name.strip(), fields=fields)
        except Exception:
            return None

    def _parse_subscription(self, data: bytes, subscriptions: Dict[int, str]):
        """Parse ADD_LOGGED_MSG: multi_id(1) + msg_id(2) + message_name(NUL-terminated)"""
        try:
            if len(data) >= 3:
                msg_id = int.from_bytes(data[1:3], 'little')
                name = data[3:].decode('utf-8', errors='replace').strip('\x00')
                subscriptions[msg_id] = name
        except Exception:
            pass

    def _parse_info_msg(self, data: bytes, info: Dict[str, str]):
        """Parse INFO message: key_type(1) + key=NUL + value"""
        try:
            text = data.decode('utf-8', errors='replace').strip('\x00')
            if '=' in text:
                key, value = text.split('=', 1)
                info[key.strip()] = value.strip()
        except Exception:
            pass

    def _parse_parameter_msg(self, data: bytes, parameters: Dict[str, Any]):
        """Parse PARAMETER message.

        PX4 ULog PARAMETER format:
          data[0] = key_len (length of key string, NOT a type enum)
          data[1 : 1+key_len] = key string (includes type prefix, e.g. "float ASPD_SCALE_1")
          data[1+key_len : ] = value (size based on type prefix)

        The key string has format: "<type> <name>" where type is like
        "float", "int32_t", "bool", "double", etc.
        """
        try:
            if len(data) < 2:
                return
            key_len = data[0]
            if key_len == 0 or 1 + key_len > len(data):
                return

            key_str = data[1:1 + key_len].decode('utf-8', errors='replace').strip('\x00')

            # Split type and name: "float ASPD_SCALE_1" -> ("float", "ASPD_SCALE_1")
            if ' ' in key_str:
                type_str, name = key_str.split(' ', 1)
                name = name.strip()
            else:
                # Fallback: no type prefix
                type_str = ''
                name = key_str

            if not name:
                return

            value_data = data[1 + key_len:]
            value = self._decode_param_by_type_str(type_str, value_data)
            if value is not None:
                parameters[name] = value
        except Exception:
            pass

    @staticmethod
    def _decode_param_by_type_str(type_str: str, raw: bytes) -> Any:
        """Decode parameter value based on the type string from the key."""
        try:
            # Strip _t suffix: int32_t -> int32
            t = type_str.rstrip('_t') if type_str.endswith('_t') else type_str

            if t in ('float',):
                return struct.unpack('f', raw[:4])[0] if len(raw) >= 4 else None
            elif t in ('double',):
                return struct.unpack('d', raw[:8])[0] if len(raw) >= 8 else None
            elif t in ('int8', 'int8_t'):
                return int.from_bytes(raw[:1], 'little', signed=True) if len(raw) >= 1 else None
            elif t in ('uint8', 'uint8_t'):
                return raw[0] if len(raw) >= 1 else None
            elif t in ('int16', 'int16_t'):
                return int.from_bytes(raw[:2], 'little', signed=True) if len(raw) >= 2 else None
            elif t in ('uint16', 'uint16_t'):
                return int.from_bytes(raw[:2], 'little') if len(raw) >= 2 else None
            elif t in ('int32', 'int32_t'):
                return int.from_bytes(raw[:4], 'little', signed=True) if len(raw) >= 4 else None
            elif t in ('uint32', 'uint32_t'):
                return int.from_bytes(raw[:4], 'little') if len(raw) >= 4 else None
            elif t in ('int64', 'int64_t'):
                return int.from_bytes(raw[:8], 'little', signed=True) if len(raw) >= 8 else None
            elif t in ('uint64', 'uint64_t'):
                return int.from_bytes(raw[:8], 'little') if len(raw) >= 8 else None
            elif t in ('bool', 'bool_t'):
                return raw[0] != 0 if len(raw) >= 1 else None
            else:
                # Unknown type, try float as fallback
                if len(raw) >= 4:
                    return struct.unpack('f', raw[:4])[0]
                return None
        except Exception:
            return None

    def _parse_field_defs(self, fields_str: str) -> List[FieldDef]:
        """Parse field definitions from format string: 'type name;type name;...'"""
        fields = []
        if not fields_str:
            return fields

        for part in fields_str.split(';'):
            part = part.strip()
            if not part:
                continue

            tokens = part.split()
            if len(tokens) < 2:
                continue

            type_str = tokens[0]
            name = tokens[1]

            # Handle arrays: type[N]
            array_size = 0
            if '[' in type_str:
                try:
                    base = type_str.split('[')[0]
                    size_str = type_str.split('[')[1].rstrip(']')
                    array_size = int(size_str)
                    type_str = base
                except (ValueError, IndexError):
                    pass

            # Strip _t suffix (uint64_t -> uint64)
            if type_str.endswith('_t'):
                type_str = type_str[:-2]

            is_padding = name.startswith('_padding')

            fields.append(FieldDef(
                type_str=type_str,
                name=name,
                array_size=array_size,
                is_padding=is_padding,
            ))

        return fields

    def _parse_data_msg(self, data: bytes,
                        formats: Dict[int, TopicFormat],
                        topics: Dict[str, List[TopicData]]):
        """Parse DATA message: msg_id(2 LE) + payload"""
        if len(data) < 2:
            return

        msg_id = int.from_bytes(data[0:2], 'little')
        if msg_id not in formats:
            return

        fmt = formats[msg_id]
        payload = data[2:]
        payload_size = len(payload)

        # Decode each field, handling truncated data gracefully
        values = {}
        field_pos = 0
        for f in fmt.non_padding_fields:
            if field_pos + f.size > payload_size:
                break

            field_bytes = payload[field_pos:field_pos + f.size]
            field_pos += f.size

            decoded = self._decode_field(f, field_bytes)
            if f.array_size > 0:
                for idx, val in enumerate(decoded):
                    values[f'{f.name}[{idx}]'] = val
            else:
                values[f.name] = decoded

        # Extract timestamp
        timestamp = 0
        if 'timestamp' in values and isinstance(values['timestamp'], int):
            timestamp = values['timestamp']
        elif 'timestamp_sample' in values and isinstance(values['timestamp_sample'], int):
            timestamp = values['timestamp_sample']

        entry = TopicData(
            timestamp=timestamp,
            topic_name=fmt.name,
            values=values,
        )

        if fmt.name not in topics:
            topics[fmt.name] = []
        topics[fmt.name].append(entry)

    def _decode_field(self, f: FieldDef, raw: bytes):
        """Decode a field value (scalar or array)"""
        if f.array_size > 0:
            elem_size = f.size // f.array_size
            return [self._decode_scalar(f.type_str, raw[i * elem_size:(i + 1) * elem_size])
                    for i in range(f.array_size)]
        else:
            return self._decode_scalar(f.type_str, raw)

    @staticmethod
    def _decode_scalar(type_str: str, raw: bytes) -> Any:
        """Decode a single scalar value from raw bytes"""
        try:
            if type_str == 'uint8':
                return raw[0]
            elif type_str == 'uint16':
                return int.from_bytes(raw, 'little')
            elif type_str == 'uint32':
                return int.from_bytes(raw, 'little')
            elif type_str == 'uint64':
                return int.from_bytes(raw, 'little')
            elif type_str == 'int8':
                return int.from_bytes(raw, 'little', signed=True)
            elif type_str == 'int16':
                return int.from_bytes(raw, 'little', signed=True)
            elif type_str == 'int32':
                return int.from_bytes(raw, 'little', signed=True)
            elif type_str == 'int64':
                return int.from_bytes(raw, 'little', signed=True)
            elif type_str == 'float':
                return struct.unpack('f', raw)[0]
            elif type_str == 'double':
                return struct.unpack('d', raw)[0]
            elif type_str == 'bool':
                return raw[0] != 0
            elif type_str == 'char':
                return raw[0] if raw else 0
            else:
                return raw.hex()
        except Exception:
            return None
