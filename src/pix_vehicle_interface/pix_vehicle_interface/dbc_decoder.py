import cantools
import os
import logging


def _to_num(val):
    """
    Safely convert a cantools decoded value to a plain Python number.
    Handles NamedSignalValue (returned for signals with VAL_ definitions)
    across all cantools versions — some make it an int subclass, others don't.
    """
    if isinstance(val, (int, float)):
        return val
    # NamedSignalValue has a numeric value — try its .value attribute first,
    # then fall back to int() cast, then 0.
    if hasattr(val, 'value'):
        return val.value
    try:
        return int(val)
    except (TypeError, ValueError):
        return 0


class DBCDecoder:
    def __init__(self, dbc_path):
        if not os.path.exists(dbc_path):
            raise FileNotFoundError(f"DBC file not found at: {dbc_path}")
        self.db = cantools.database.load_file(dbc_path)

    def decode_message(self, frame_id, data):
        """
        Decode raw CAN data into a dict of plain numeric signals.
        decode_choices=False forces raw integer values instead of
        NamedSignalValue objects (which crash int() on older cantools).
        """
        try:
            msg = self.db.get_message_by_frame_id(frame_id)
            # decode_choices=False → always returns int/float, never NamedSignalValue
            decoded_raw = msg.decode(data, decode_choices=False)
            # Belt-and-suspenders: run every value through _to_num
            decoded = {k: _to_num(v) for k, v in decoded_raw.items()}
            return msg.name, decoded
        except KeyError:
            # frame_id not in DBC — ignore silently (e.g. ultrasonic sensor frames)
            return None, None
        except Exception as e:
            logging.error(f"Error decoding CAN frame ID 0x{frame_id:03X}: {e}")
            return None, None

