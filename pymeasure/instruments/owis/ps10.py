import time
from pymeasure.instruments import Instrument


class PS10(Instrument):
    """
    Driver for the PS 10-32 Position Control Unit.
    Supports basic setup, querying status, and controlling motion.
    """

    motor_type = Instrument.control(
        "?MOTYPE1", "MOTYPE1=%d",
        """Control the motor type. (int).

            0 = DC brush
            1 = stepper motor Open Loop
        """,
        cast=int,
    )

    limit_switch_mask = Instrument.control(
        "?SMK1", "SMK1=%d",
        """Control the limit switch mask (int).

            This command activates or deactivates the limit and break
            switches. If a limit switch is approached, the movement is
            stopped abruptly and the motor is shut off.

            Bit sequence : <MAXSTOP, MAXDEC, MINDEC, MINSTOP>.
        """,
        cast=int,
    )

    hysteresis = Instrument.measurement(
        "?HYST1",
        """Measure the refernce switch hysteresis. (int).

            After a reference motion has been terminated successfully, the
            hysteresis of the switch can be read out with this command.
            (The value is correct only, if none of the reference/limit
            switches is active any more)
        """,
        cast=int,
    )

    position_target = Instrument.control(
        "?PSET1", "PSET1=%d",
        """Control the target position (int).
        """,
        cast=int,
    )

    position_counter = Instrument.control(
        "?CNT1", "CNT1=%d",
        """Control the current position counter (int).
        """,
        cast=int,
    )

    velocity_max = Instrument.control(
        "?PVEL1", "PVEL1=%d",
        """Set maximum positioning velocity for the axis (int).

            Used for the trapezoidal profile.
        """,
        cast=int,
    )

    acceleration = Instrument.control(
        "?ACC1", "ACC1=%d",
        """Set acceleration (= run-up ramp) for the axis (int).

            Used for all modes (trapezoidal, velocity mode, etc.).
        """,
        cast=int,
    )

    def __init__(self, adapter, name="PS 10-32 Motor Controller", **kwargs):
        super().__init__(
            adapter,
            name,
            write_termination="\r",
            read_termination="\r",
            timeout=5000,  # in ms
            **kwargs
        )
        self.query_delay = 0.02
        self.adapter.query_delay = self.query_delay

    def write(self, command, **kwargs):
        """Write a string command to the instrument appending `write_termination`.

        This is an override of the Instrument.write() method to include a small delay.

        :param command: command string to be sent to the instrument
        :param kwargs: Keyword arguments for the adapter.
        """
        self.adapter.write(command, **kwargs)
        self.wait_for(self.query_delay)  # Ensure a small delay after the command

    def initialize(self):
        """
        Initializes the motor controller and prepares it for operation.
        """
        self.write("INIT1")  # Initialize the axis (assuming axis 1)

    def start_reference_motion(self, mode):
        """
        Starts a reference motion for the axis.
        
        :param mode: Integer defining the reference mode. Valid values:
            - 0: Search next index impulse and stop.
            - 1: Approach reference switch and stop.
            - 2: Approach reference switch, search next index impulse and stop.
            - 3: Mode 0, additionally set actual position to 0.
            - 4: Mode 1, additionally set actual position to 0.
            - 5: Mode 2, additionally set actual position to 0.
            - 6: Approach maximum reference switch, approach minimum reference switch, set position to 0.
            - 7: Approach minimum reference switch, approach maximum reference switch, set position to 0.
        """
        if mode not in range(8):
            raise ValueError("Invalid mode. Must be an integer between 0 and 7.")
        self.write(f"REF1={mode}")

    def move_to_position(self, position):
        """
        Moves to a specific position.
        :param position: Target position (integer, absolute or relative).
        """
        self.position_target = position
        self.write("PGO1")  # Start motion

    def stop_motion(self):
        """
        Stops motion with deceleration.
        """
        self.write("STOP1")

    def wait_for_motion_complete(self, timeout=3):
        """
        Waits until the axis motion is complete by polling the axis state.

        :param timeout: Maximum time to wait in seconds (default: 60 seconds).
        :return: True if the motion is complete, False if timeout occurs.
        """
        start_time = time.time()

        while True:
            # Query the axis state
            axis_state = self.query_axis_state()

            # Check if the axis is not in a motion state (e.g., "R" means ready)
            if axis_state.get("R", False):  # "R" means ready state (motion complete)
                return True
            elif axis_state.get("T", False):  # "T" means axis is positioning in trapezoidal profile
                start_time = time.time()  # Still moving, reset timer
            elif axis_state.get("P", False):  # "T" means reference motion is in progress
                start_time = time.time()  # Still moving, reset timer
            elif axis_state.get("F", False):  # "F" means axis is releasing a limit switch
                start_time = time.time()  # Still moving, reset timer

            # Timeout check
            if time.time() - start_time > timeout:
                print(f"Timeout: Axis motion did not complete within {timeout} seconds.")
                return False

            # Sleep for a small duration before polling again
            time.sleep(10*self.query_delay)

    def query_position(self):
        """
        Queries the current position.
        :return: Current position as integer.
        """
        return int(self.ask("?CNT1"))

    def save_parameters(self):
        """
        Saves the current configuration to EEPROM.
        """
        self.write("SAVEPARA")

    def reset(self):
        """
        Resets the main board.
        """
        self.write("RESETMB")

    def query_axis_state(self):
        """
        Queries the current state of the axis.

        :return: A dictionary describing the axis state. Possible states:
            - "I": Axis is not initialized.
            - "O": Axis is disabled.
            - "R": Axis is initialized and ready.
            - "T": Axis is positioning in trapezoidal profile.
            - "V": Axis is operating in velocity mode.
            - "P": Reference motion is in progress.
            - "F": Axis is releasing a limit switch.
            - "L": Axis disabled after hardware limit switch error.
            - "B": Axis stopped after brake switch error.
            - "A": Axis disabled after limit switch error.
            - "M": Axis disabled after motion-controller error.
            - "Z": Axis disabled after timeout error.
            - "H": Phase initialization active (step motor axis).
            - "U": Axis not enabled.
            - "E": Axis disabled after motion error.
            - "?": Unknown error state.
        """
        state_codes = {
            "I": "Not initialized",
            "O": "Disabled",
            "R": "Initialized and ready",
            "T": "Positioning in trapezoidal profile",
            "V": "Operating in velocity mode",
            "P": "Reference motion in progress",
            "F": "Releasing a limit switch",
            "L": "Disabled after hardware limit switch error",
            "B": "Stopped after brake switch error",
            "A": "Disabled after limit switch error",
            "M": "Disabled after motion-controller error",
            "Z": "Disabled after timeout error",
            "H": "Phase initialization active (step motor axis)",
            "U": "Not enabled",
            "E": "Disabled after motion error",
            "?": "Unknown state",
        }
        response = self.ask("?ASTAT")
        return {code: state_codes.get(code, "Unknown state") for code in response}

    def query_message_buffer(self):
        """
        Queries the message buffer for any error messages.

        :return: A string message describing the current state of the message buffer.
            Possible messages include:
            - "00 NO MESSAGE AVAILABLE"
            - "01 PARAMETER BEFORE EQUAL WRONG"
            - "02 AXIS NUMBER WRONG"
            - "03 PARAMETER AFTER EQUAL WRONG"
            - "04 PARAMETER AFTER EQUAL RANGE"
            - "05 WRONG COMMAND ERROR"
            - "06 REPLY IMPOSSIBLE"
            - "07 AXIS IS IN WRONG STATE"
        """
        message_codes = {
            "00": "NO MESSAGE AVAILABLE",
            "01": "PARAMETER BEFORE EQUAL WRONG",
            "02": "AXIS NUMBER WRONG",
            "03": "PARAMETER AFTER EQUAL WRONG",
            "04": "PARAMETER AFTER EQUAL RANGE",
            "05": "WRONG COMMAND ERROR",
            "06": "REPLY IMPOSSIBLE",
            "07": "AXIS IS IN WRONG STATE",
        }
        response = self.ask("?MSG")
        return {response: message_codes.get(response, "Unknown state")}

    def is_reference_valid(self):
        """
        Checks if the reference position for the axis is valid.
        
        :return: True if the reference position is valid, False otherwise.
        """
        response = self.ask("?REFST1")  # Query the reference status for axis 1
        return response.strip() == "1"  # '1' means valid, '0' means invalid

    def set_motor_on(self):
        """
        Sets the motor to be ON.
        """
        self.write("MON1")

    def set_motor_off(self):
        """
        Sets the motor to be OFF.
        """
        self.write("MOFF1")

    def release_limit_switches(self):
        """
        Moves the stage off a limit switch.

        Release limit switch(es) of the axis. After a drive has moved onto a
        limit switch (MINSTOP, MAXSTOP) or brake switch (MINDEC, MAXDEC), the
        active switch(es) can be released using this command. The direction
        of the movement is automatically decided according to whether a
        positive or negative limit or break switch is activated.
        """
        self.write("EFREE1")

    def query_limit_switch_status(self):
        """
        Queries the status of the limit switches and power stage feedback for the axis.

        :param axis: Axis number (default: 1).
        :return: Dictionary with the status of the limit switches and power stage feedback.
            - Bit 0: MINSTOP
            - Bit 1: MINDEC
            - Bit 2: MAXDEC
            - Bit 3: MAXSTOP
            - Bit 4: Motor power-stage error
        """
        response = self.ask(f"?ESTAT1")
        status_bits = int(response.strip())  # Convert the response to an integer for bit-level analysis

        # Extract the status of each bit
        status = {
            "MINSTOP": bool(status_bits & 0b00001),
            "MINDEC": bool(status_bits & 0b00010),
            "MAXDEC": bool(status_bits & 0b00100),
            "MAXSTOP": bool(status_bits & 0b01000),
            "Motor power-stage error": bool(status_bits & 0b10000),
        }
        return status

    def query_error_memory(self, max_polling=20):
        """
        Queries the error memory and retrieves the most recent error messages.
        Polls up to a specified number of times (default 20) until the error memory is cleared (error code 0).

        :param max_polling: Maximum number of times to poll the error memory (default: 20).
        :return: List of error codes (four-digit ASCII numbers).
        """
        errors = []
        for _ in range(max_polling):
            response = self.ask("?ERR")  # Query the error memory
            error_code = response.strip()  # Strip any whitespace

            if error_code == '0':  # No more errors
                break

            errors.append(error_code)  # Add the error code to the list

        return errors


    def query_emergency_stop_status(self):
        """
        Queries the current state of the emergency stop input.

        :return: Integer representing the state of the emergency stop input (1 or 0).
            - 1: Emergency stop is active
            - 0: Emergency stop is not active
        """
        response = self.ask("?EMERGINP")
        return int(response.strip())  # Convert the response to an integer (1 or 0)


    def measure_counts_per_revolution(self):
        """
        Measures the number of counts for a full revolution using the reference switch (MINSTOP).

        :return: Number of counts for a full revolution.
        """
        # Reference the motor using mode 4 (set actual position to 0)
        print('Referencing')
        self.start_reference_motion(4)  # Reference with mode 4
        self.wait_for_motion_complete()

        # Move off the reference switch in the positive direction (clear the switch)
        print('Moving off MINSTOP')
        self.move_to_position(4*self.hysteresis)  # Move off the reference switch in the positive direction
        self.wait_for_motion_complete()
        self.limit_switch_mask = 0b0001  # Activate MINSTOP

        # Move in the positive direction and check if MINSTOP is activated
        print('Searching for MINSTOP')
        self.move_to_position(2**31 - 1)  # Move to maximum positive position (signed 32 bit number)
        while True:
            status = self.query_limit_switch_status()  # Query the current status of limit switches
            if status['MINSTOP'] is True:  # If MINSTOP is set, reference switch is triggered
                break
            self.wait_for(10*self.query_delay)

        # Move off the MINSTOP switch in the positive direction (clear the switch)
        print('Releasing limit switch')
        self.initialize()
        self.wait_for(time.sleep(1))  # Initialization takes a little while
        self.release_limit_switches()
        self.wait_for_motion_complete()
        self.limit_switch_mask = 0b0000  # Deactivate MINSTOP
        print('Moving off MINSTOP')
        self.move_to_position(self.position_counter + 4*self.hysteresis)  # Move off the reference switch in the positive direction
        self.wait_for_motion_complete()

        # Reference the motor using mode 1 (do not set actual position)
        print('Referencing')
        self.start_reference_motion(1)  # Reference with mode 1
        self.wait_for_motion_complete()

        # Read actual position to get counts per revolution (coarse)
        self.counts_per_revolution = self.position_counter  # Query position counter
        print(f"Coarse counts per revolution: {self.counts_per_revolution}")

        # - Repeat measurement without E-stop - #
        print('# Repeating measurement without E-stop')

        # Reference the motor using mode 4 (set actual position to 0)
        print('Referencing')
        self.start_reference_motion(4)  # Reference with mode 4
        self.wait_for_motion_complete()

        # Move in the positive direction one 1.1 revolutions
        print('Moving 1.1 revolutions')
        self.move_to_position(1.1*self.counts_per_revolution)  # Move to maximum positive position (signed 32 bit number)
        self.wait_for_motion_complete()

        # Reference the motor using mode 1 (do not set actual position)
        print('Referencing')
        self.start_reference_motion(1)  # Reference with mode 1
        self.wait_for_motion_complete()

        # Read actual position to get counts per revolution (fine)
        self.counts_per_revolution = self.position_counter  # Query position counter
        print(f"Fine counts per revolution: {self.counts_per_revolution}")

        return self.counts_per_revolution

    def decode_one_wire_id(self, one_wire_id):
        """
        Best guess at decoding Owis rotation stage ID parameters
        """
        fields = one_wire_id.split('|')
        decoded = {
            "Part Number": fields[0],
            "Type": fields[1],
            "Serial Number": fields[2],
            "Unknown": fields[3],
            "Pitch": fields[4],
            "Gear reduction ratio": fields[5],
            "Movement type": fields[6],
            "Motor type": fields[7],
            "Nom. Current [A]": fields[8],
            "Fullsteps/rev": fields[9],
            "Encoder lines": fields[10],
            "Resolution [nm]/Glass scale": fields[11],
            "Signal period [um]/Glass scale": fields[12],
            "Switches and brake configuration": fields[13],
        }
        return decoded

    def query_one_wire_id(self):
        """
        Queries the one-wire ID for a axis1 by iteratively reading 16-byte blocks.

        :return: The full one-wire ID as a concatenated string.
        """
        one_wire_id = ""
        offset = 0  # Start with the first block

        while True:
            # Query the 16-byte block starting at the current offset
            response = self.ask(f"?READOWID1={offset}")

            # Break the loop if an empty string is returned
            if not response:
                break

            # Append the response to the full ID
            one_wire_id += response
            offset += 16  # Move to the next 16-byte block

        return one_wire_id

    def query_one_wire_unique_block(self):
        """
        Queries the one-wire unique block for a specific axis.

        :return: The one-wire unique block as a string.
        """
        return self.ask(f"?READOWUB1").strip()