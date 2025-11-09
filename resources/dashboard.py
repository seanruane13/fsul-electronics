import time
import threading
import streamlit as st
import can
from can_parser import DTAECUParser
from sim_can_sender import DTASwinS40Simulator  # reuse your generator class

VIRTUAL_CHANNEL = "fsul"  # both TX and RX attach to this channel

def _make_virtual_bus():
    """Create a python-can in-process virtual bus, new API first, fallback to old."""
    try:
        # python-can >= 4.2
        return can.Bus(interface="virtual", channel=VIRTUAL_CHANNEL)
    except TypeError:
        # older python-can
        return can.interface.Bus(bustype="virtual", channel=VIRTUAL_CHANNEL)

def start_sim_sender(hz: int, stop_flag: dict):
    """
    Start a background thread that publishes simulated frames.
    IMPORTANT: uses its OWN virtual bus handle (TX) so the RX handle can receive.
    """
    sim = DTASwinS40Simulator(frequency=hz)
    tx_bus = _make_virtual_bus()

    def loop():
        try:
            while not stop_flag.get("stop", False):
                t0 = time.time()
                for cid in sim.can_ids:
                    data = sim.gen_bytes(cid)  # bytes (8)
                    msg = can.Message(arbitration_id=cid, data=data, is_extended_id=True)
                    try:
                        tx_bus.send(msg)
                    except Exception:
                        return
                dt = time.time() - t0
                time.sleep(max(0.0, 1.0 / hz - dt))
        finally:
            try:
                tx_bus.shutdown()
            except Exception:
                pass

    th = threading.Thread(target=loop, daemon=True)
    th.start()
    return th

def reader_thread(parser: DTAECUParser, stop_flag: dict):
    """
    Background CAN read loop.
    Avoids any direct Streamlit calls to prevent ScriptRunContext warnings.
    """
    # If a bus was injected (e.g., virtual bus), skip connect()
    if parser.bus is None:
        try:
            connected = parser.connect()
        except Exception:
            connected = False
        if not connected:
            print("Failed to connect to CAN bus.")
            return
    else:
        print("Using injected CAN bus for RX")

    # Use a local counter instead of touching st.session_state
    frames_rx = 0

    try:
        while not stop_flag.get("stop", False):
            msg = parser.bus.recv(timeout=0.1)
            if msg is not None:
                parser.parse_message(msg)
                frames_rx += 1
    finally:
        try:
            if parser.bus is not None:
                parser.bus.shutdown()
        except Exception:
            pass
        parser.bus = None
        print(f"Reader thread stopped after receiving {frames_rx} frames.")


def _cleanup_previous():
    """Stop any existing threads and shut down any existing bus cleanly."""
    if "stop_flag" in st.session_state:
        st.session_state["stop_flag"]["stop"] = True
        time.sleep(0.2)

    old_parser = st.session_state.get("parser")
    if getattr(old_parser, "bus", None):
        try:
            old_parser.bus.shutdown()
        except Exception:
            pass

    st.session_state["reader_thread"] = None
    st.session_state["sim_thread"] = None

def main():
    st.set_page_config(page_title="FSUL ECU Dashboard", layout="wide")
    st.title("FSUL ECU Live Dashboard")

    # Sidebar config
    st.sidebar.header("Connection")
    mode = st.sidebar.radio("Source", ["Simulator (virtual)", "Real ECU (can0)"], index=0)

    st.sidebar.header("Controls")
    auto_refresh = st.sidebar.checkbox("Auto refresh", value=True)
    refresh_rate = st.sidebar.slider("Refresh interval (ms)", 50, 1000, 100, 50)

    # Reconnect button forces a fresh setup of the selected mode
    if st.sidebar.button("Reconnect"):
        _cleanup_previous()
        st.session_state["initialized"] = False

    # Initialize (or re-initialize) based on the selected mode
    if not st.session_state.get("initialized"):
        _cleanup_previous()
        st.session_state["stop_flag"] = {"stop": False}
        st.session_state["frames_rx"] = 0  # reset counter

        if mode.startswith("Simulator"):
            # RX handle for the parser (separate from TX handle)
            rx_bus = _make_virtual_bus()

            # Create parser normally, then inject the RX bus
            parser = DTAECUParser(channel=VIRTUAL_CHANNEL, bustype="virtual", bitrate=1_000_000)
            parser.bus = rx_bus  # inject RX bus

            st.session_state["parser"] = parser

            # Start reader (RX) and simulator (TX) on the SAME channel but DIFFERENT handles
            rt = threading.Thread(
                target=reader_thread,
                args=(parser, st.session_state["stop_flag"]),
                daemon=True,
            )
            rt.start()
            st.session_state["reader_thread"] = rt

            st.session_state["sim_thread"] = start_sim_sender(
                hz=10, stop_flag=st.session_state["stop_flag"]
            )
        else:
            # Real ECU path (requires real Linux SocketCAN on e.g. can0)
            parser = DTAECUParser(channel="can0", bustype="socketcan", bitrate=1_000_000)
            st.session_state["parser"] = parser

            rt = threading.Thread(
                target=reader_thread,
                args=(parser, st.session_state["stop_flag"]),
                daemon=True,
            )
            rt.start()
            st.session_state["reader_thread"] = rt

        st.session_state["initialized"] = True

    # Helpers
    def fmt(x, digits=1):
        try:
            return f"{float(x):.{digits}f}"
        except Exception:
            return "-"

    # Rolling charts (can be removed if you don't want them)
    if "rpm_hist" not in st.session_state:
        st.session_state["rpm_hist"] = []
        st.session_state["spd_hist"] = []
        st.session_state["wt_hist"] = []
        st.session_state["bt_hist"] = []
        st.session_state["ts_hist"] = []

    # Single placeholder so elements update in place
    placeholder = st.empty()

    def draw_once():
        parser: DTAECUParser = st.session_state["parser"]
        ed = parser.engine_data  # live dataclass

        with placeholder.container():
            # Create columns INSIDE the placeholder so they refresh instead of append
            col1, col2, col3, col4 = st.columns(4)
            col5, col6, col7, col8 = st.columns(4)

            col1.metric("Engine RPM", f"{ed.rpm} rpm")
            col2.metric("Speed", f"{fmt(ed.speed)} kph")
            col3.metric("TPS", f"{fmt(ed.tps)} %")
            col4.metric("Manifold P", f"{fmt(ed.manifold_pressure)} kPa")
            col5.metric("Water Temp", f"{fmt(ed.water_temp)} °C")
            col6.metric("Air Temp", f"{fmt(ed.air_temp)} °C")
            col7.metric("Battery", f"{fmt(ed.battery_voltage)} V")
            col8.metric("Inj Time", f"{fmt(ed.injection_time, 2)} ms")

            st.caption("Reader thread active ✅")

            # Traces (optional)
            st.subheader("Traces (last ~10s)")
            st.session_state["rpm_hist"].append(ed.rpm)
            st.session_state["spd_hist"].append(ed.speed)
            st.session_state["wt_hist"].append(ed.water_temp)
            st.session_state["bt_hist"].append(ed.battery_voltage)
            st.session_state["ts_hist"].append(ed.tps)

            for key in ["rpm_hist", "spd_hist", "wt_hist", "bt_hist", "ts_hist"]:
                if len(st.session_state[key]) > 500:
                    st.session_state[key] = st.session_state[key][-500:]

            c1, c2, c3 = st.columns(3)
            c1.line_chart(st.session_state["rpm_hist"], height=180)
            c2.line_chart(st.session_state["spd_hist"], height=180)
            c3.line_chart(st.session_state["wt_hist"], height=180)
            d1, d2 = st.columns(2)
            d1.line_chart(st.session_state["bt_hist"], height=180)
            d2.line_chart(st.session_state["ts_hist"], height=180)

    if auto_refresh:
        while True:
            draw_once()
            time.sleep(refresh_rate / 1000.0)
    else:
        if st.button("Refresh now"):
            draw_once()
        else:
            draw_once()

if __name__ == "__main__":
    main()
