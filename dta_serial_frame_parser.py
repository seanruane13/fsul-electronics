# dta_serial_frame_parser.py 
import time 
 

PORT = 'COM12'      # change 
BAUD = 57600 
HEADER = bytes([0xD0, 0xD0])   # change to the header you set in DTASwin 
TIMEOUT = 1.0 
 

def checksum_ok(payload, chk): 
    # common simple checksum = sum(payload) & 0xFF 
    return (sum(payload) & 0xFF) == chk 
 

def find_header(buf, header): 
    idx = buf.find(header) 
    return idx 
 

# Try to import pyserial. A common cause of the error
# "module 'serial' has no attribute 'Serial'" is that a
# different package/module named `serial` is installed or a
# local `serial.py` file is shadowing the real pyserial package.
try:
    import serial
    _has_serial = hasattr(serial, 'Serial')
except Exception as _e:
    # Keep going but mark that pyserial.Serial is not available.
    serial = None
    _has_serial = False
    print(f"Warning: failed to import 'serial' module: {_e}")

def serial_debug_info():
    """Print helpful diagnostics about which `serial` module is loaded."""
    try:
        import importlib, inspect
        if serial is None:
            print("serial module: None")
            return
        print('serial module repr:', repr(serial))
        print('serial.__file__:', getattr(serial, '__file__', None))
        print('serial package:', getattr(serial, '__package__', None))
        print('has Serial attribute:', hasattr(serial, 'Serial'))
        if hasattr(serial, 'Serial'):
            print('Serial repr:', repr(serial.Serial))
    except Exception as _e:
        print('Error while printing serial debug info:', _e)

class MockSerial:
    def __init__(self, port, baudrate, timeout):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.buffer = bytearray()

    def read(self, size):
        # Simulate reading by returning empty bytes
        return b''

    def close(self):
        pass

def main(): 
    if not _has_serial:
        print("Warning: pyserial's Serial not found. Falling back to MockSerial.")
        print("Common causes: a local file named 'serial.py' or the 'serial' package (not pyserial) is installed.")
        print("To fix: install pyserial (pip install pyserial) and ensure no local 'serial.py' shadows it.\n")
        serial_debug_info()
        ser = MockSerial(PORT, BAUD, TIMEOUT)
    else:
        ser = serial.Serial(PORT, BAUD, timeout=0.1) 
    buf = bytearray() 
    print(f'Listening {PORT} @ {BAUD}, header={HEADER.hex()}') 
    try: 
        while True: 
            b = ser.read(128) 
            if b: 
                buf.extend(b) 
            # try to find header 
            i = find_header(buf, HEADER) 
            while i != -1 and len(buf) > i + len(HEADER) + 1: 
                start = i + len(HEADER) 
                # length byte after header? 
                length = buf[start] 
                if len(buf) >= start + 1 + length + 1: 
                    payload = bytes(buf[start+1:start+1+length]) 
                    chk = buf[start+1+length] 
                    ok = checksum_ok(payload, chk) 
                    print(f'FRAME len={length} chk=0x{chk:02X} OK={ok} PAYLOAD={payload.hex()}') 
                    # remove consumed bytes 
                    del buf[:start+1+length+1] 
                    # search for another header 
                    i = find_header(buf, HEADER) 
                else: 
                    # need more bytes 
                    break 
            # avoid uncontrolled buffer growth 
            if len(buf) > 4096: 
                buf = buf[-1024:] 
    except KeyboardInterrupt: 
        print('Stopped') 
    finally: 
        ser.close() 
 

if __name__ == '__main__': 
    main()
