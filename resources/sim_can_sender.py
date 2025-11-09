import time
import random
import struct
import can

class DTASwinS40Simulator:
    def __init__(self, bitrate=1_000_000, frequency=10):
        self.bitrate = bitrate
        self.frequency = frequency
        self.interval = 1.0 / frequency

        # IDs (we’ll send as extended-id frames; your parser just looks at IDs)
        self.can_ids = [
            0x2000, 0x2001, 0x2002, 0x2003, 0x2004,
            0x2005, 0x2006, 0x2007, 0x2008, 0x2009
        ]

        # state
        self.engine_running = True
        self.current_gear = 1
        self.vehicle_speed = 0.0
        self.rpm = 800
        self.manifold_pressure = 30
        self.water_temp = 85
        self.air_temp = 25
        self.oil_temp = 90
        self.battery_voltage = 138   # x10 => 13.8V
        self.fuel_consumption = 120  # x10 => 12.0 L/100km

    @staticmethod
    def pack_s16(x: int) -> bytes:
        return struct.pack('<h', int(x))

    def gen_0x2000(self) -> bytes:
        # RPM, TPS, Water Temp, Air Temp (all s16 LE)
        if self.engine_running:
            self.rpm = int(800 + random.randint(0, 200) + abs((time.time() % 10) - 5) * 100)
            self.rpm = max(800, min(8000, self.rpm))
        tps = random.randint(10, 90)
        self.water_temp = 80 + random.randint(-5, 20)
        self.air_temp   = 20 + random.randint(-5, 15)
        return (self.pack_s16(self.rpm) +
                self.pack_s16(tps) +
                self.pack_s16(self.water_temp) +
                self.pack_s16(self.air_temp))

    def gen_0x2001(self) -> bytes:
        # Manifold kPa, Lambda1 x1000, Speed x10 kph, Oil pressure kPa
        self.manifold_pressure = max(20, min(100, int(self.rpm/100) + random.randint(-5, 5)))
        lambda1 = 1000 + random.randint(-50, 50)
        self.vehicle_speed = max(0.0, min(200.0, (self.rpm/40.0) + random.randint(-5, 5)))
        speed_x10 = int(self.vehicle_speed * 10)
        oil_kpa = int(max(100, min(500, self.rpm/20 + 200 + random.randint(-10, 10))))
        return (self.pack_s16(self.manifold_pressure) +
                self.pack_s16(lambda1) +
                self.pack_s16(speed_x10) +
                self.pack_s16(oil_kpa))

    def gen_0x2002(self) -> bytes:
        # Fuel P kPa, Oil Temp C, Battery x10 V, Fuel L/h x10
        fuel_kpa = 300 + random.randint(-20, 20)
        self.oil_temp = int(80 + self.rpm/100 + random.randint(-5, 5))
        batt_x10 = int(self.battery_voltage)
        fuel_lh_x10 = int(50 + self.rpm/100 + random.randint(-10, 10))
        return (self.pack_s16(fuel_kpa) +
                self.pack_s16(self.oil_temp) +
                self.pack_s16(batt_x10) +
                self.pack_s16(fuel_lh_x10))

    def gen_0x2003(self) -> bytes:
        # Current gear, Position (unused here), Advance x10, Inj time x100
        v = self.vehicle_speed
        if v > 80:   self.current_gear = 6
        elif v > 60: self.current_gear = max(self.current_gear,5)
        elif v > 40: self.current_gear = max(self.current_gear,4)
        elif v > 25: self.current_gear = max(self.current_gear,3)
        elif v > 10: self.current_gear = max(self.current_gear,2)
        elif v <  5: self.current_gear = 1

        advance_x10 = 150 + random.randint(-10, 10)     # ~15.0°
        inj_time_x100 = int(1000 + self.rpm/10 + random.randint(-50, 50))  # ms x100
        self.fuel_consumption = max(50, 200 - v/2 + random.randint(-20, 20))  # L/100km x10
        gear_pos = self.current_gear  # placeholder
        return (self.pack_s16(self.current_gear) +
                self.pack_s16(gear_pos) +
                self.pack_s16(advance_x10) +
                self.pack_s16(inj_time_x100))

    def gen_0x2004(self) -> bytes:
        ana1 = 2500 + random.randint(-100, 100)
        ana2 = 1500 + random.randint(-100, 100)
        ana3 =  800 + random.randint(-50, 50)
        cam_adv_x10 = 100 + random.randint(-10, 10)
        return (self.pack_s16(ana1) +
                self.pack_s16(ana2) +
                self.pack_s16(ana3) +
                self.pack_s16(cam_adv_x10))

    def gen_0x2005(self) -> bytes:
        cam_target_x10 = 120 + random.randint(-5, 5)
        cam_pwm_x10    = 500 + random.randint(-20, 20)
        crank_err      = random.randint(0, 2)
        cam_err        = random.randint(0, 1)
        return (self.pack_s16(cam_target_x10) +
                self.pack_s16(cam_pwm_x10) +
                self.pack_s16(crank_err) +
                self.pack_s16(cam_err))

    def gen_0x2006(self) -> bytes:
        cam2_adv_x10    = 110 + random.randint(-5, 5)
        cam2_target_x10 = 115 + random.randint(-5, 5)
        cam2_pwm_x10    = 480 + random.randint(-20, 20)
        ext_5v_mv       = 5000 + random.randint(-100, 100)
        return (self.pack_s16(cam2_adv_x10) +
                self.pack_s16(cam2_target_x10) +
                self.pack_s16(cam2_pwm_x10) +
                self.pack_s16(ext_5v_mv))

    def gen_0x2007(self) -> bytes:
        inj_duty = int(min(95, max(5, (self.rpm/100) + random.randint(0, 10))))
        l_pid_target_x10 = 1000 + random.randint(-20, 20)
        l_pid_adj_x10    = random.randint(-50, 50)

        ecu_switches = 0
        if self.vehicle_speed > 0:  ecu_switches |= 0x01
        if self.vehicle_speed > 80: ecu_switches |= 0x02
        ecu_switches |= 0x04
        if random.random() > 0.8:   ecu_switches |= 0x08
        ecu_switches |= 0x10
        if self.water_temp > 95:    ecu_switches |= 0x20

        return (self.pack_s16(inj_duty) +
                self.pack_s16(l_pid_target_x10) +
                self.pack_s16(l_pid_adj_x10) +
                self.pack_s16(ecu_switches))

    def gen_0x2008(self) -> bytes:
        base = int(self.vehicle_speed * 10)
        rd  = base + random.randint(-2, 2)
        rud = base + random.randint(-3, 1)
        ld  = base + random.randint(-1, 3)
        lud = base + random.randint(-2, 2)
        return (self.pack_s16(rd) +
                self.pack_s16(rud) +
                self.pack_s16(ld) +
                self.pack_s16(lud))

    def gen_0x2009(self) -> bytes:
        right_lambda_x1000 = 980 + random.randint(-30, 30)
        return (self.pack_s16(right_lambda_x1000) +
                b'\x00\x00\x00\x00\x00\x00')

    def gen_bytes(self, can_id: int) -> bytes:
        return {
            0x2000: self.gen_0x2000,
            0x2001: self.gen_0x2001,
            0x2002: self.gen_0x2002,
            0x2003: self.gen_0x2003,
            0x2004: self.gen_0x2004,
            0x2005: self.gen_0x2005,
            0x2006: self.gen_0x2006,
            0x2007: self.gen_0x2007,
            0x2008: self.gen_0x2008,
            0x2009: self.gen_0x2009,
        }[can_id]()

def main(channel='vcan0', bustype='socketcan', hz=10):
    sim = DTASwinS40Simulator(frequency=hz)
    bus = can.interface.Bus(channel=channel, bustype=bustype, bitrate=1_000_000)

    print(f"[SIM] Sending frames on {bustype}:{channel} at {hz} Hz. Ctrl+C to stop.")
    try:
        while True:
            t0 = time.time()
            for cid in sim.can_ids:
                data = sim.gen_bytes(cid)
                msg = can.Message(
                    arbitration_id=cid,
                    data=data,
                    is_extended_id=True  # send as 29-bit
                )
                bus.send(msg)
            dt = time.time() - t0
            time.sleep(max(0.0, sim.interval - dt))
    except KeyboardInterrupt:
        print("\n[SIM] Stopped.")
    finally:
        bus.shutdown()

if __name__ == "__main__":
    main()
