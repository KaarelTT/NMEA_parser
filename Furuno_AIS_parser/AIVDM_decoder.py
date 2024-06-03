from pyais import decode
from pyais.exceptions import InvalidNMEAMessageException
import time


class ShipInfo:
    def __init__(self, mmsi, position, heading, speed, timestamp):
        self.mmsi = mmsi
        self.position = position
        self.heading = heading
        self.speed = speed
        self.timestamp = timestamp


# Dictionary to store ships by their MMSI
ships = {}
# Temporary storage for multi-part messages
multi_part_messages = {}


def process_ais_message(message):
    global ships, multi_part_messages

    try:
        # Ensure the message is in bytes
        if isinstance(message, str):
            message = message.encode('utf-8')

        # Split the message into parts
        message_parts = message.decode('utf-8').split(',')

        if message_parts[0] != "!AIVDM":
            return None  # Not an AIVDM message

        total_parts = int(message_parts[1])
        part_number = int(message_parts[2])
        sequence_id = message_parts[3]
        message_content = message_parts[5]

        # Initialize the sequence ID if not present
        if sequence_id not in multi_part_messages:
            multi_part_messages[sequence_id] = [None] * total_parts

        # Store the part
        multi_part_messages[sequence_id][part_number - 1] = message_content

        # Check if we have received all parts
        if None not in multi_part_messages[sequence_id]:
            full_message = "!AIVDM,1,1,,A," + "".join(multi_part_messages[sequence_id]) + ",0*00"
            del multi_part_messages[sequence_id]
            decoded_message = decode(full_message.encode('utf-8'))
        else:
            return None  # Wait for more parts to complete the message

        # Extract required information using attributes instead of subscripts
        mmsi = decoded_message.mmsi
        position = (getattr(decoded_message, 'lat', None), getattr(decoded_message, 'lon', None))
        heading = getattr(decoded_message, 'heading', None)
        speed = getattr(decoded_message, 'sog', None)
        timestamp = time.time()

        # Update the ship information in the dictionary
        ships[mmsi] = ShipInfo(mmsi, position, heading, speed, timestamp)

        # Clean up old entries
        current_time = time.time()
        for mmsi in list(ships.keys()):
            if current_time - ships[mmsi].timestamp > 600:
                del ships[mmsi]

        return ships

    except InvalidNMEAMessageException as e:
        print(f"Invalid NMEA message: {message}")
    except Exception as e:
        print(f"Error processing message: {e}")

    return None
