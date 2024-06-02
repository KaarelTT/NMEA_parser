from pyais import decode
from pyais.exceptions import InvalidNMEAMessageException
from pyais.messages import NMEAMessage

# Define a data structure to hold ship information
class ShipInfo:
    def __init__(self, mmsi, position, heading, speed):
        self.mmsi = mmsi
        self.position = position
        self.heading = heading
        self.speed = speed

# Temporary storage for multi-part messages
multi_part_messages = {}

def process_ais_message(message):
    global multi_part_messages

    try:
        nmea_message = NMEAMessage(message)

        # Check if the message is multi-part
        if nmea_message.is_multi:
            part_number = nmea_message.part_number
            total_parts = nmea_message.total_parts
            sequence_id = nmea_message.sequence_id

            # Initialize the sequence ID if not present
            if sequence_id not in multi_part_messages:
                multi_part_messages[sequence_id] = [None] * total_parts

            # Store the part
            multi_part_messages[sequence_id][part_number - 1] = nmea_message

            # Check if we have received all parts
            if None not in multi_part_messages[sequence_id]:
                full_message = "".join([part.data for part in multi_part_messages[sequence_id]])
                del multi_part_messages[sequence_id]
                decoded_message = decode(NMEAMessage(full_message))
            else:
                return None  # Wait for more parts to complete the message
        else:
            decoded_message = decode(nmea_message)

        # Extract required information
        mmsi = decoded_message['mmsi']
        position = (decoded_message['x'], decoded_message['y'])
        heading = decoded_message['heading']
        speed = decoded_message['sog']

        ship_info = ShipInfo(mmsi, position, heading, speed)

        return ship_info

    except InvalidNMEAMessageException as e:
        print(f"Invalid NMEA message: {message}")
    except Exception as e:
        print(f"Error processing message: {e}")

    return None
