import cantools
import os
import logging

class DBCEncoder:
    def __init__(self, dbc_path):
        if not os.path.exists(dbc_path):
            raise FileNotFoundError(f"DBC file not found at: {dbc_path}")
        self.db = cantools.database.load_file(dbc_path)
        
    def encode_message(self, message_name, signals):
        """
        Encode signals into raw CAN message payload and return (arbitration_id, payload).
        Calculates checksums dynamically if the message has a checksum field.
        """
        try:
            msg = self.db.get_message_by_name(message_name)
            
            # Find checksum signal name in this message if it exists
            checksum_sig = None
            for sig in msg.signals:
                if sig.name.lower().startswith('checksum'):
                    checksum_sig = sig.name
                    break
                    
            if checksum_sig:
                signals[checksum_sig] = 0
                
            # Perform encoding using cantools
            data = msg.encode(signals)
            data_arr = bytearray(data)
            
            # PIX checksum: XOR of bytes 0-6, placed in byte 7.
            # Reference: Whale component (data[0]^data[1]^...^data[6])
            if checksum_sig:
                checksum = 0
                for b in data_arr[:7]:
                    checksum ^= b
                data_arr[7] = checksum
                
            return msg.frame_id, bytes(data_arr)
        except Exception as e:
            logging.error(f"Error encoding CAN message {message_name}: {e}")
            return None, None
