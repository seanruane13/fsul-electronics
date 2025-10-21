import can
import struct
import time
from dataclasses import dataclass
from typing import Dict, Any, Optional

@dataclass
class EngineData:
    """Data class to store parsed engine parameters"""
    # 0x2000 packet
    rpm: int = 0
    tps: float = 0.0
    water_temp: float = 0.0
    air_temp: float = 0.0
    
    # 0x2001 packet
    manifold_pressure: float = 0.0
    lambda1: float = 0.0
    lambda2: float = 0.0
    speed: float = 0.0
    oil_pressure: float = 0.0
    
    # 0x2002 packet
    fuel_pressure: float = 0.0
    oil_temp: float = 0.0
    battery_voltage: float = 0.0
    fuel_consumption_lh: float = 0.0
    
    # 0x2003 packet
    current_gear: int = 0
    gear_position: int = 0
    advance: float = 0.0
    injection_time: float = 0.0
    fuel_consumption_100km: float = 0.0
    
    # 0x2004 packet
    ana1: float = 0.0
    ana2: float = 0.0
    ana3: float = 0.0
    cam_advance: float = 0.0
    
    # 0x2005 packet
    cam_target: float = 0.0
    cam_pwm: float = 0.0
    cam_pwm_percentage: float = 0.0
    crank_errors: int = 0
    cam_errors: int = 0
    
    # 0x2006 packet
    cam2_advance: float = 0.0
    cam2_target: float = 0.0
    cam2_pwm: float = 0.0
    external_5v: float = 0.0
    
    # 0x2007 packet
    inj_duty_cycle: float = 0.0
    lambda_pid_target: float = 0.0
    lambda_pid_adj: float = 0.0
    ecu_switches: int = 0
    
    # 0x2008 packet
    rd_speed: float = 0.0
    r_ud_speed: float = 0.0
    ld_speed: float = 0.0
    l_ud_speed: float = 0.0
    
    # 0x2009 packet
    right_lambda: float = 0.0


class DTAECUParser:
    """DTA S60 ECU CAN Bus Parser"""
    
    def __init__(self, channel: str = 'can0', bustype: str = 'socketcan', bitrate: int = 1000000):
        """
        Initialize the CAN bus interface
        
        Args:
            channel: CAN interface channel (e.g., 'can0', 'vcan0')
            bustype: CAN bus type ('socketcan', 'pcan', etc.)
            bitrate: CAN bus bitrate (1 MBd for DTA)
        """
        self.channel = channel
        self.bustype = bustype
        self.bitrate = bitrate
        self.bus = None
        self.engine_data = EngineData()
        
    def connect(self):
        """Connect to CAN bus"""
        try:
            self.bus = can.interface.Bus(
                channel=self.channel,
                bustype=self.bustype,
                bitrate=self.bitrate
            )
            print(f"Connected to CAN bus {self.channel} at {self.bitrate} baud")
            return True
        except Exception as e:
            print(f"Error connecting to CAN bus: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from CAN bus"""
        if self.bus:
            self.bus.shutdown()
            self.bus = None
            print("Disconnected from CAN bus")
    
    def parse_signed_16bit(self, data: bytes, offset: int) -> int:
        """Parse signed 16-bit integer (little endian)"""
        return struct.unpack('<h', data[offset:offset+2])[0]
    
    def parse_ecu_switches(self, switch_value: int) -> Dict[str, bool]:
        """Parse ECU switches bitmask"""
        return {
            'launch_button_pressed': bool(switch_value & 0x0001),
            'launch_active': bool(switch_value & 0x0002),
            'traction_on': bool(switch_value & 0x0004),
            'traction_wet': bool(switch_value & 0x0008),
            'fuel_pump_on': bool(switch_value & 0x0010),
            'fan_output_on': bool(switch_value & 0x0020),
            # Bits 6-15 are reserved
        }
    
    def parse_message(self, msg: can.Message):
        """Parse incoming CAN message"""
        if len(msg.data) != 8:
            return
        
        arbitration_id = msg.arbitration_id
        
        try:
            if arbitration_id == 0x2000:
                # Packet 0x2000: RPM, TPS, Water Temp, Air Temp
                self.engine_data.rpm = self.parse_signed_16bit(msg.data, 0)
                self.engine_data.tps = self.parse_signed_16bit(msg.data, 2)
                self.engine_data.water_temp = self.parse_signed_16bit(msg.data, 4)
                self.engine_data.air_temp = self.parse_signed_16bit(msg.data, 6)
                
            elif arbitration_id == 0x2001:
                # Packet 0x2001: Manifold Pressure, Lambda1, Lambda2, Speed, Oil Pressure
                self.engine_data.manifold_pressure = self.parse_signed_16bit(msg.data, 0)
                self.engine_data.lambda1 = self.parse_signed_16bit(msg.data, 2) / 1000.0
                self.engine_data.lambda2 = self.parse_signed_16bit(msg.data, 4) / 1000.0
                self.engine_data.speed = self.parse_signed_16bit(msg.data, 6) / 10.0
                # Oil pressure seems to be missing from this packet based on spec
                
            elif arbitration_id == 0x2002:
                # Packet 0x2002: Fuel Pressure, Oil Temp, Battery, Fuel Consumption
                self.engine_data.fuel_pressure = self.parse_signed_16bit(msg.data, 0)
                self.engine_data.oil_temp = self.parse_signed_16bit(msg.data, 2)
                self.engine_data.battery_voltage = self.parse_signed_16bit(msg.data, 4) / 10.0
                self.engine_data.fuel_consumption_lh = self.parse_signed_16bit(msg.data, 6) / 10.0
                
            elif arbitration_id == 0x2003:
                # Packet 0x2003: Gear, Position, Advance, Injection, Fuel Consumption
                self.engine_data.current_gear = self.parse_signed_16bit(msg.data, 0)
                self.engine_data.gear_position = self.parse_signed_16bit(msg.data, 2)
                self.engine_data.advance = self.parse_signed_16bit(msg.data, 4) / 10.0
                self.engine_data.injection_time = self.parse_signed_16bit(msg.data, 6) / 100.0
                # Fuel consumption per 100km might be in different bytes
                
            elif arbitration_id == 0x2004:
                # Packet 0x2004: Analog inputs, Cam Advance
                self.engine_data.ana1 = self.parse_signed_16bit(msg.data, 0)
                self.engine_data.ana2 = self.parse_signed_16bit(msg.data, 2)
                self.engine_data.ana3 = self.parse_signed_16bit(msg.data, 4)
                self.engine_data.cam_advance = self.parse_signed_16bit(msg.data, 6) / 10.0
                
            elif arbitration_id == 0x2005:
                # Packet 0x2005: Cam control and errors
                self.engine_data.cam_target = self.parse_signed_16bit(msg.data, 0) / 10.0
                self.engine_data.cam_pwm = self.parse_signed_16bit(msg.data, 2) / 10.0
                self.engine_data.cam_pwm_percentage = self.parse_signed_16bit(msg.data, 4) / 10.0
                # Errors might be packed differently
                self.engine_data.crank_errors = self.parse_signed_16bit(msg.data, 6)
                
            elif arbitration_id == 0x2006:
                # Packet 0x2006: Cam2 control and external 5V
                self.engine_data.cam2_advance = self.parse_signed_16bit(msg.data, 0) / 10.0
                self.engine_data.cam2_target = self.parse_signed_16bit(msg.data, 2) / 10.0
                self.engine_data.cam2_pwm = self.parse_signed_16bit(msg.data, 4) / 10.0
                self.engine_data.external_5v = self.parse_signed_16bit(msg.data, 6)
                
            elif arbitration_id == 0x2007:
                # Packet 0x2007: Injection and Lambda PID
                self.engine_data.inj_duty_cycle = self.parse_signed_16bit(msg.data, 0)
                self.engine_data.lambda_pid_target = self.parse_signed_16bit(msg.data, 2) / 10.0
                self.engine_data.lambda_pid_adj = self.parse_signed_16bit(msg.data, 4) / 10.0
                self.engine_data.ecu_switches = self.parse_signed_16bit(msg.data, 6)
                
            elif arbitration_id == 0x2008:
                # Packet 0x2008: Wheel speeds
                self.engine_data.rd_speed = self.parse_signed_16bit(msg.data, 0) / 10.0
                self.engine_data.r_ud_speed = self.parse_signed_16bit(msg.data, 2) / 10.0
                self.engine_data.ld_speed = self.parse_signed_16bit(msg.data, 4) / 10.0
                self.engine_data.l_ud_speed = self.parse_signed_16bit(msg.data, 6) / 10.0
                
            elif arbitration_id == 0x2009:
                # Packet 0x2009: Right Lambda
                self.engine_data.right_lambda = self.parse_signed_16bit(msg.data, 0) / 1000.0
                
        except Exception as e:
            print(f"Error parsing message ID {hex(arbitration_id)}: {e}")
    
    def get_ecu_switches_status(self) -> Dict[str, bool]:
        """Get parsed ECU switches status"""
        return self.parse_ecu_switches(self.engine_data.ecu_switches)
    
    def print_live_data(self):
        """Print formatted live data"""
        switches = self.get_ecu_switches_status()
        
        print("\n" + "="*50)
        print("DTA S60 ECU LIVE DATA")
        print("="*50)
        print(f"Engine RPM: {self.engine_data.rpm:6d} rpm")
        print(f"TPS:        {self.engine_data.tps:6.1f} %")
        print(f"Water Temp: {self.engine_data.water_temp:6.1f} °C")
        print(f"Air Temp:   {self.engine_data.air_temp:6.1f} °C")
        print(f"Manifold P: {self.engine_data.manifold_pressure:6.1f} kPa")
        print(f"Lambda 1:   {self.engine_data.lambda1:6.3f}")
        print(f"Speed:      {self.engine_data.speed:6.1f} kph")
        print(f"Battery:    {self.engine_data.battery_voltage:6.1f} V")
        print(f"Inj Time:   {self.engine_data.injection_time:6.2f} ms")
        print(f"Inj Duty:   {self.engine_data.inj_duty_cycle:6.1f} %")
        print("\nECU Switches:")
        for switch, status in switches.items():
            print(f"  {switch}: {'ON' if status else 'OFF'}")
        print("="*50)
    
    def start_live_monitor(self, print_interval: float = 1.0):
        """
        Start live monitoring of CAN bus data
        
        Args:
            print_interval: Interval in seconds between data display updates
        """
        if not self.connect():
            return
        
        print("Starting live CAN bus monitoring...")
        print("Press Ctrl+C to stop")
        
        last_print_time = time.time()
        
        try:
            while True:
                # Read CAN message with timeout
                msg = self.bus.recv(timeout=0.1)
                
                if msg is not None:
                    self.parse_message(msg)
                
                # Print data at specified interval
                current_time = time.time()
                if current_time - last_print_time >= print_interval:
                    self.print_live_data()
                    last_print_time = current_time
                    
        except KeyboardInterrupt:
            print("\nStopping live monitor...")
        finally:
            self.disconnect()


# Example usage
if __name__ == "__main__":
    # Create parser instance
    # For real CAN interface (Linux):
    # parser = DTAECUParser(channel='can0', bustype='socketcan')
    
    # For virtual CAN interface (testing):
    parser = DTAECUParser(channel='vcan0', bustype='socketcan')
    
    # Start live monitoring
    parser.start_live_monitor(print_interval=1.0)
    
    # Alternative: Manual message processing example
    """
    if parser.connect():
        try:
            while True:
                msg = parser.bus.recv(timeout=1.0)
                if msg:
                    parser.parse_message(msg)
                    # Do something with parsed data...
        except KeyboardInterrupt:
            pass
        finally:
            parser.disconnect()
    """
    
# LINUX SocketCAN
# sudo modprobe can
# sudo modprobe can_raw
# sudo modprobe vcan  # for virtual CAN testing

# # Set up virtual CAN for testing
# sudo ip link add dev vcan0 type vcan
# sudo ip link set up vcan0

# # For real CAN interface (adjust interface name)
# sudo ip link set can0 type can bitrate 1000000
# sudo ip link set up can0