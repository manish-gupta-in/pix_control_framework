import cantools
import os
import logging

class DBCDecoder:
    def __init__(self, dbc_path):
        if not os.path.exists(dbc_path):
            raise FileNotFoundError(f"DBC file not found at: {dbc_path}")
        self.db = cantools.database.load_file(dbc_path)
        
    def decode_message(self, frame_id, data):
        """
        Decode raw CAN data into a dictionary of signals and return (message_name, signals).
        """
        try:
            msg = self.db.get_message_by_frame_id(frame_id)
            decoded = msg.decode(data)
            return msg.name, decoded
        except KeyError:
            # message ID not in DBC, ignore
            return None, None
        except Exception as e:
            logging.error(f"Error decoding message ID {frame_id}: {e}")
            return None, None
