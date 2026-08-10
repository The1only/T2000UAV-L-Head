#!/usr/bin/env python3
"""
T2000UAV-L serial test app

Protocol:
    STX x=<command> ETX

Where:
    STX = 0x02
    ETX = 0x03

Serial:
    9600 baud
    8 data bits
    no parity
    1 stop bit
    XON/XOFF enabled
"""

import argparse
import time
import serial


STX = 0x02
ETX = 0x03


def make_packet(command: str, value: str) -> bytes:
    """
    Build packet: STX + x=<command> + ETX

    Example:
        make_packet("d", "?") -> b'\\x02d=?\\x03'
        make_packet("c", "2212") -> b'\\x02c=2212\\x03'
    """
    body = f"{command}={value}".encode("ascii")
    return bytes([STX]) + body + bytes([ETX])


def bytes_to_hex(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def read_response(ser: serial.Serial, timeout: float = 2.0) -> bytes:
    """
    Read until ETX or timeout.

    Some functions, such as reply annunciator, may return '*' without a full
    STX/ETX packet, so this function also returns any raw data received.
    """
    end_time = time.time() + timeout
    data = bytearray()

    while time.time() < end_time:
        waiting = ser.in_waiting

        if waiting:
            chunk = ser.read(waiting)
            data.extend(chunk)

            if ETX in chunk:
                break
        else:
            time.sleep(0.01)

    return bytes(data)


def clean_response(data: bytes) -> str:
    """
    Remove STX/ETX for display and decode as ASCII.
    """
    cleaned = data.replace(bytes([STX]), b"").replace(bytes([ETX]), b"")

    try:
        return cleaned.decode("ascii", errors="replace")
    except Exception:
        return repr(cleaned)


def send_command(ser: serial.Serial, command: str, value: str, timeout: float = 2.0):
    packet = make_packet(command, value)

    print()
    print(f"Sending command: {command}={value}")
    print(f"TX ASCII body : {command}={value}")
    print(f"TX HEX        : {bytes_to_hex(packet)}")

    ser.reset_input_buffer()
    ser.write(packet)
    ser.flush()

    response = read_response(ser, timeout=timeout)

    if response:
        print(f"RX HEX        : {bytes_to_hex(response)}")
        print(f"RX TEXT       : {clean_response(response)}")
    else:
        print("RX            : No response / timeout")


def interactive_mode(ser: serial.Serial):
    print()
    print("T2000UAV-L interactive test mode")
    print("Enter commands like:")
    print("  d=?")
    print("  c=2212")
    print("  s=c")
    print("  v=?")
    print("  z=?")
    print()
    print("Type 'help' for command list.")
    print("Type 'quit' to exit.")

    while True:
        line = input("\nT2000> ").strip()

        if not line:
            continue

        if line.lower() in ("q", "quit", "exit"):
            break

        if line.lower() == "help":
            print_help()
            continue

        if "=" not in line:
            print("Invalid format. Use for example: d=? or c=2212")
            continue

        command, value = line.split("=", 1)
        command = command.strip()
        value = value.strip()

        if len(command) != 1:
            print("Command must be one character, for example: d")
            continue

        send_command(ser, command, value)


def print_help():
    print()
    print("Available commands from manual:")
    print()
    print("  a=?       Query altitude")
    print("  a=1234F   Write altitude")
    print()
    print("  c=?       Query assigned code")
    print("  c=2212    Set assigned code")
    print()
    print("  d=?       Query altitude source")
    print("  d=g       Set altitude source to Gillham")
    print("  d=s       Set altitude source to Serial")
    print()
    print("  e=?       Query encoder power mode")
    print("  e=o       Encoder power always on")
    print("  e=c       Encoder power mode C only")
    print()
    print("  i=?       Query ident status")
    print("  i=s       Squawk ident")
    print()
    print("  p=?       Ping")
    print()
    print("  r=?       Query reply annunciator")
    print("  r=y       Enable reply annunciator")
    print("  r=n       Disable reply annunciator")
    print()
    print("  s=?       Query operational mode")
    print("  s=t       Standby")
    print("  s=a       Mode A")
    print("  s=c       Mode C")
    print()
    print("  v=?       Query bus voltage")
    print("  v=1       Set bus voltage mode to 14V")
    print("  v=2       Set bus voltage mode to 28V")
    print()
    print("  z=?       Query software revision")


def main():
    parser = argparse.ArgumentParser(description="T2000UAV-L RS232 test app")
    parser.add_argument(
        "port",
        help="Serial port, for example COM3 on Windows or /dev/ttyUSB0 on Linux",
    )
    parser.add_argument(
        "--command",
        "-c",
        help="Single command to send, for example 'd=?' or 'c=2212'",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Read timeout in seconds, default 2.0",
    )

    args = parser.parse_args()

    with serial.Serial(
        port=args.port,
        baudrate=9600,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        xonxoff=True,      # XON/XOFF enabled
        rtscts=False,
        dsrdtr=False,
        timeout=0.1,
        write_timeout=2.0,
    ) as ser:
        print(f"Opened serial port: {args.port}")
        print("Settings: 9600, 8N1, XON/XOFF enabled")

        if args.command:
            if "=" not in args.command:
                raise ValueError("Command must be in format x=value, for example d=?")

            command, value = args.command.split("=", 1)

            if len(command) != 1:
                raise ValueError("Command must be one character, for example d")

            send_command(ser, command, value, timeout=args.timeout)
        else:
            interactive_mode(ser)


if __name__ == "__main__":
    main()

