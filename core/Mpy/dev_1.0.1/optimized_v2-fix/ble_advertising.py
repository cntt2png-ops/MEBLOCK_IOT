# ble_advertising.py
# Helpers for generating and decoding BLE advertising payloads.

from micropython import const
import struct
import bluetooth

_ADV_TYPE_FLAGS = const(0x01)
_ADV_TYPE_NAME = const(0x09)
_ADV_TYPE_UUID16_COMPLETE = const(0x03)
_ADV_TYPE_UUID32_COMPLETE = const(0x05)
_ADV_TYPE_UUID128_COMPLETE = const(0x07)
_ADV_TYPE_APPEARANCE = const(0x19)

_ADV_MAX_PAYLOAD = const(31)


def _as_bytes(value):
    if value is None:
        return None
    if isinstance(value, bytes):
        return value
    if isinstance(value, bytearray):
        return bytes(value)
    return str(value).encode("utf-8")


def _field_len(value):
    return 2 + len(value)


def _append_field(payload, adv_type, value):
    payload.extend(struct.pack("BB", len(value) + 1, adv_type))
    payload.extend(value)


def _uuid_field_type(raw):
    ln = len(raw)
    if ln == 2:
        return _ADV_TYPE_UUID16_COMPLETE
    if ln == 4:
        return _ADV_TYPE_UUID32_COMPLETE
    if ln == 16:
        return _ADV_TYPE_UUID128_COMPLETE
    return None


def _name_budget(flags_len, services_len, appearance_len):
    budget = _ADV_MAX_PAYLOAD - (flags_len + services_len + appearance_len)
    # name field adds 2 bytes header + N bytes payload
    budget -= 2
    if budget < 0:
        budget = 0
    return budget


def fit_name_for_advertising(name, services=None, appearance=0):
    name_b = _as_bytes(name)
    if not name_b:
        return b""

    services_len = 0
    if services:
        for uuid in services:
            raw = bytes(uuid)
            if _uuid_field_type(raw) is not None:
                services_len += _field_len(raw)

    appearance_len = _field_len(struct.pack("<h", appearance)) if appearance else 0
    max_name = _name_budget(3, services_len, appearance_len)

    if len(name_b) <= max_name:
        return name_b

    # keep UTF-8 valid when trimming
    trimmed = name_b[:max_name]
    while trimmed:
        try:
            trimmed.decode("utf-8")
            return trimmed
        except Exception:
            trimmed = trimmed[:-1]
    return b""


def advertising_payload(limited_disc=False, br_edr=False, name=None, services=None, appearance=0):
    payload = bytearray()

    flags = struct.pack("B", (0x01 if limited_disc else 0x02) + (0x18 if br_edr else 0x04))
    _append_field(payload, _ADV_TYPE_FLAGS, flags)

    if services:
        for uuid in services:
            raw = bytes(uuid)
            adv_type = _uuid_field_type(raw)
            if adv_type is not None:
                _append_field(payload, adv_type, raw)

    if appearance:
        _append_field(payload, _ADV_TYPE_APPEARANCE, struct.pack("<h", appearance))

    name_b = fit_name_for_advertising(name, services=services, appearance=appearance)
    if name_b:
        _append_field(payload, _ADV_TYPE_NAME, name_b)

    if len(payload) > _ADV_MAX_PAYLOAD:
        raise ValueError("advertising payload too large")

    return payload


def decode_field(payload, adv_type):
    i = 0
    result = []
    plen = len(payload)
    while i + 1 < plen:
        field_len = payload[i]
        if field_len == 0:
            break
        field_end = i + field_len + 1
        if field_end > plen:
            break
        if payload[i + 1] == adv_type:
            result.append(payload[i + 2:field_end])
        i = field_end
    return result


def decode_name(payload):
    fields = decode_field(payload, _ADV_TYPE_NAME)
    if not fields:
        return ""
    try:
        return fields[0].decode("utf-8")
    except Exception:
        return ""


def decode_services(payload):
    services = []
    for code in (_ADV_TYPE_UUID16_COMPLETE, _ADV_TYPE_UUID32_COMPLETE, _ADV_TYPE_UUID128_COMPLETE):
        for raw in decode_field(payload, code):
            try:
                services.append(bluetooth.UUID(raw))
            except Exception:
                pass
    return services


def demo():
    payload = advertising_payload(
        name="micropython",
        services=[bluetooth.UUID(0x181A), bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")],
    )
    print(payload)
    print(decode_name(payload))
    print(decode_services(payload))


if __name__ == "__main__":
    demo()
