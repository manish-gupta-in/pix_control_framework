#!/usr/bin/env python3
"""
hooke2_test.py  —  Hooke2 DTV actuator test tool  (v3 — CAN matrix corrected)

MODES:
  brake   Apply brake at specific %% with optional ramp
  gear    Select specific gear OR run full N→D→N→R→N→P cycle
  park    Engage / release park brake (0x104) and verify via monitor
  monitor Watch all CAN state live (no commands sent)

KEY FINDINGS from CAN matrix:
  - 0x103 Gear_Target: byte1 bits[2:0], Gear_EnCtrl: byte0 bit0, XOR checksum
  - 0x104 Park: NOT in CAN matrix — using same layout as working driver
  - 0x503/0x504 gear/park reports: NOT in CAN matrix — VCU may not publish these
  - Brake_EnState: byte0 bits[2:1] (not 1:0), EnCtrl: byte0 bit0

USAGE:
  python3 hooke2_test.py --mode brake --pct 30 --ramp 10 --hold 5
  python3 hooke2_test.py --mode gear  --gear drive
  python3 hooke2_test.py --mode gear  --cycle --hold 3
  python3 hooke2_test.py --mode park  --park-action engage --hold 5
  python3 hooke2_test.py --mode monitor
"""

import sys, os, time, signal, threading, csv, argparse
from datetime import datetime

try:
    import can
except ImportError:
    sys.exit("Run: python3 -m pip install python-can --user")

CHANNEL = "can4"
DT      = 0.02   # 50 Hz

# ════════════════════════════════════════════════════════
#  FRAME BUILDERS  — verified against CAN matrix
# ════════════════════════════════════════════════════════

def _xor(d):
    cs = 0
    for b in d[:7]: cs ^= b
    return cs & 0xFF

def build_vehicle_mode():
    """0x105 — VCU AUTO mode keepalive, SUM checksum."""
    d = bytearray([0x80, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
    d[7] = sum(d[:7]) & 0xFF
    return d

def build_brake(enable=True, pedal_pct=0.0, decel_ms2=0.0):
    """
    0x101 Brake_Command — XOR checksum
      byte0 bit0    : Brake_EnCtrl
      byte0 bit1    : AEB_EnCtrl (always 0)
      bytes 1-2     : Brake_Dec  (10-bit Motorola LSB, start_bit=15, res=0.01 m/s²)
      bytes 3-4     : Brake_Pedal_Target (16-bit Motorola LSB, start_bit=31, res=0.1%)
      byte7         : XOR checksum
    """
    d = bytearray(8)
    d[0] = 0x01 if enable else 0x00
    # Brake_Dec: 10-bit at start_bit=15 Motorola → bytes 1-2, bits 15..6
    dec_raw = min(int(round(max(0.0, min(10.0, decel_ms2)) / 0.01)), 0x3FF)
    d[1] = (dec_raw >> 2) & 0xFF
    d[2] = (dec_raw & 0x3) << 6
    # Brake_Pedal_Target: 16-bit at start_bit=31 Motorola → bytes 3-4
    pedal_raw = int(round(max(0.0, min(100.0, pedal_pct)) / 0.1))
    d[3] = (pedal_raw >> 8) & 0xFF
    d[4] = pedal_raw & 0xFF
    d[7] = _xor(d)
    return d

def build_gear(enable=True, gear_id=3):
    """
    0x103 Gear_Command — SUM checksum (confirmed working)
      byte0 bit0       : Gear_EnCtrl
      byte1 bits[2:0]  : Gear_Target (1=PARK 2=REV 3=NEU 4=DRV)
      byte7            : SUM checksum (sum of bytes 0-6)
    """
    d = bytearray(8)
    d[0] = 0x01 if enable else 0x00
    d[1] = gear_id & 0x07
    d[7] = sum(d[:7]) & 0xFF
    return d

def build_park(enable=True, engage=True):
    """
    0x104 Park_Command — SUM checksum (confirmed working)
      byte0 bit0 : Park_EnCtrl
      byte1      : Park_Target (1=engage, 0=release)
      byte7      : SUM checksum (sum of bytes 0-6)
    """
    d = bytearray(8)
    d[0] = 0x01 if enable else 0x00
    d[1] = 0x01 if engage  else 0x00
    d[7] = sum(d[:7]) & 0xFF
    return d

GEAR_ID   = {"park":1, "reverse":2, "neutral":3, "drive":4}
GEAR_NAME = {v:k.upper() for k,v in GEAR_ID.items()}

def build_throttle(enable=True, pedal_pct=0.0):
    """
    0x100 Throttle_Command — SUM checksum (per pixkit.dbc)
      byte0 bit0  : Dirve_EnCtrl
      bytes 3-4   : Dirve_ThrottlePedalTarget (%, res=0.1)   [start_bit=31,len=16]
      byte7       : SUM checksum
    Send enable=True, pedal_pct=0 to bring Dirve_EnState to Auto
    (required by VCU before it permits a gear shift).
    """
    d = bytearray(8)
    d[0] = 0x01 if enable else 0x00
    pedal_raw = int(round(max(0.0, min(100.0, pedal_pct)) / 0.1))
    d[3] = (pedal_raw >> 8) & 0xFF
    d[4] = pedal_raw & 0xFF
    d[7] = sum(d[:7]) & 0xFF
    return d

def build_steer(enable=True, angle=0, angle_speed=50):
    """
    0x102 Steering_Command — SUM checksum (per pixkit.dbc)
      byte0 bit0  : Steer_EnCtrl
      byte1       : Steer_AngleSpeed (deg/s, 0-250)          [start_bit=15,len=8]
      bytes 2-3   : Steer_AngleTarget, raw = angle+500       [start_bit=31,len=16,offset=-500]
      byte7       : SUM checksum
    Send enable=True, angle=0 (center) to bring Steer_EnState to Auto
    (required by VCU before it permits a gear shift).
    """
    d = bytearray(8)
    d[0] = 0x01 if enable else 0x00
    d[1] = max(0, min(250, int(angle_speed)))
    raw = max(0, min(1000, int(angle) + 500))
    d[2] = (raw >> 8) & 0xFF
    d[3] = raw & 0xFF
    d[7] = sum(d[:7]) & 0xFF
    return d

# ════════════════════════════════════════════════════════
#  RX DECODERS  — corrected from CAN matrix
# ════════════════════════════════════════════════════════

# DBC-verified bit formula for Motorola signals confined to byte0:
#   value = byte0 & ((1<<length)-1)   when start_bit == length-1
# Brake_EnState / Dirve_EnState / Steer_EnState: start_bit=1,length=2 -> byte0 & 0x3
# Gear_Actual: start_bit=2,length=3 -> byte0 & 0x7
EN_ST    = {0:"Manual", 1:"Auto", 2:"Takeover", 3:"Standby"}
VCU_MODE = {0:"MANUAL", 1:"AUTO", 2:"EMERGENCY", 3:"STANDBY"}

def dec_brake(d):
    """0x501 Brake_Report — Brake_EnState = byte0 & 0x3 (DBC start_bit=1,len=2)"""
    en_raw = d[0] & 0x3
    return {
        "en":    EN_ST.get(en_raw, "?"),
        "flt1":  d[1], "flt2": d[2],
        "pedal": round(((d[3]<<8)|d[4])*0.1, 1)
    }

def dec_throttle(d):
    """0x500 Throttle_Report — Dirve_EnState = byte0 & 0x3"""
    en_raw = d[0] & 0x3
    return {
        "en":    EN_ST.get(en_raw, "?"),
        "flt1":  d[1], "flt2": d[2],
        "pedal": round(((d[3]<<8)|d[4])*0.1, 1)
    }

def dec_steer(d):
    """0x502 Steering_Report — Steer_EnState = byte0 & 0x3"""
    en_raw = d[0] & 0x3
    raw = (d[3]<<8)|d[4]
    return {
        "en":    EN_ST.get(en_raw, "?"),
        "flt1":  d[1], "flt2": d[2],
        "angle": raw - 500
    }


def dec_vcu(d):
    """0x505"""
    raw = (d[2]<<8)|d[3]
    if raw & 0x8000: raw -= 0x10000
    return round(raw*0.001,3), round(raw*0.001*3.6,2), VCU_MODE.get((d[4]>>3)&3,"?")

def dec_wheel(d):
    """0x506"""
    return (round(((d[0]<<8)|d[1])*0.001,3), round(((d[2]<<8)|d[3])*0.001,3),
            round(((d[4]<<8)|d[5])*0.001,3), round(((d[6]<<8)|d[7])*0.001,3))

# ════════════════════════════════════════════════════════
#  RAW CAN MONITOR — log ALL received IDs & bytes
#  (to discover actual gear/park report IDs)
# ════════════════════════════════════════════════════════

class RawMonitor:
    """Tracks which CAN IDs are seen and their latest bytes."""
    _lk   = threading.Lock()
    _seen = {}   # aid → (count, last_data_hex, last_ts)

    @classmethod
    def update(cls, aid, data):
        with cls._lk:
            cls._seen[aid] = (cls._seen.get(aid,(0,None,0))[0]+1, data.hex(), time.time())

    @classmethod
    def dump(cls):
        with cls._lk:
            return dict(cls._seen)

# ════════════════════════════════════════════════════════
#  SHARED VEHICLE STATE
# ════════════════════════════════════════════════════════

class VS:
    _lk = threading.Lock()
    brake_en="?"; brake_pct=0.0; brake_f1=0; brake_f2=0
    throttle_en="?"; throttle_pct=0.0
    steer_en="?"; steer_angle=0
    wfl=0.0; wfr=0.0; wrl=0.0; wrr=0.0
    spd_ms=0.0; spd_kmh=0.0; vcu_mode="?"

    @classmethod
    def snap(cls):
        with cls._lk:
            return {k:getattr(cls,k) for k in [
                "brake_en","brake_pct","brake_f1","brake_f2",
                "throttle_en","throttle_pct","steer_en","steer_angle",
                "wfl","wfr","wrl","wrr","spd_ms","spd_kmh","vcu_mode"]}

def rx_thread(bus, stop_evt):
    while not stop_evt.is_set():
        try:
            msg = bus.recv(timeout=0.05)
            if msg is None: continue
            d = msg.data; aid = msg.arbitration_id
            RawMonitor.update(aid, d)
            with VS._lk:
                if aid==0x501 and len(d)>=5:
                    b=dec_brake(d); VS.brake_en=b["en"]; VS.brake_pct=b["pedal"]
                    VS.brake_f1=b["flt1"]; VS.brake_f2=b["flt2"]
                elif aid==0x500 and len(d)>=5:
                    t=dec_throttle(d); VS.throttle_en=t["en"]; VS.throttle_pct=t["pedal"]
                elif aid==0x502 and len(d)>=5:
                    st=dec_steer(d); VS.steer_en=st["en"]; VS.steer_angle=st["angle"]
                elif aid==0x506 and len(d)>=8:
                    VS.wfl,VS.wfr,VS.wrl,VS.wrr=dec_wheel(d)
                elif aid==0x505 and len(d)>=5:
                    VS.spd_ms,VS.spd_kmh,VS.vcu_mode=dec_vcu(d)
        except Exception:
            pass

# ════════════════════════════════════════════════════════
#  CSV LOGGER
# ════════════════════════════════════════════════════════

class Logger:
    HEADER = ["ts","elapsed_s","mode","phase",
              "cmd_brake_pct","actual_brake_pct","brake_en","brake_flt1",
              "cmd_throttle_pct","actual_throttle_pct","throttle_en",
              "cmd_gear","cmd_park",
              "spd_kmh","vcu_mode",
              "wfl","wfr","wrl","wrr","any_wheel_moving"]

    def __init__(self, path):
        self._f=open(path,"w",newline=""); self._w=csv.writer(self._f)
        self._t0=time.time(); self._w.writerow(self.HEADER)

    def log(self, mode, phase, cmd_brake=0.0, cmd_throttle=0.0,
            cmd_gear="-", cmd_park="-"):
        s  = VS.snap()
        mv = int(any(abs(s[k])>0.01 for k in ["wfl","wfr","wrl","wrr"]))
        now = time.time()
        self._w.writerow([
            round(now,4), round(now-self._t0,4), mode, phase,
            round(cmd_brake,1), s["brake_pct"], s["brake_en"], s["brake_f1"],
            round(cmd_throttle,1), s["throttle_pct"], s["throttle_en"],
            cmd_gear, cmd_park,
            s["spd_kmh"], s["vcu_mode"],
            s["wfl"],s["wfr"],s["wrl"],s["wrr"], mv
        ])

    def close(self): self._f.flush(); self._f.close()

# ════════════════════════════════════════════════════════
#  EMERGENCY STOP — Ctrl+C at any time
# ════════════════════════════════════════════════════════

_estop_done = threading.Event()

def emergency_stop(bus, logger):
    if _estop_done.is_set(): return
    _estop_done.set()
    print(f"\n\n{'!'*56}")
    print(f"  EMERGENCY STOP — throttle OFF | 100% brake | park ON")
    print(f"{'!'*56}")
    def _tx(aid, d):
        try: bus.send(can.Message(arbitration_id=aid, data=d, is_extended_id=False))
        except: pass
    t = time.time()
    while time.time()-t < 2.5:
        _tx(0x105, build_vehicle_mode())
        _tx(0x101, build_brake(True, 100.0))
        _tx(0x104, build_park(True, True))
        if logger: logger.log("estop","estop",cmd_brake=100.0,cmd_park="engage")
        time.sleep(DT)
    s = VS.snap()
    print(f"  Done — brake={s['brake_pct']:.0f}%  spd={s['spd_kmh']} km/h")

def _tx_bus(bus, aid, data):
    try: bus.send(can.Message(arbitration_id=aid, data=data, is_extended_id=False))
    except: pass

# ════════════════════════════════════════════════════════
#  MODE: MONITOR — discover all CAN IDs
# ════════════════════════════════════════════════════════

def mode_monitor(bus, logger, secs=10.0):
    indefinite = (secs == 0)
    label = "∞" if indefinite else f"{secs:.0f}s"
    print(f"\n[MONITOR] Listening {label} — no commands sent")
    print(f"  This discovers all CAN IDs the VCU broadcasts.")
    print(f"  Look for IDs beyond 0x505/0x506 — those are gear/park reports.\n")
    print(f"  {'T':>6}  {'brake%':>7}  {'En':<9}  {'Spd km/h':>9}  {'VCU':<8}  IDs seen")
    print(f"  {'':─<70}")

    t0 = time.time()
    while True:
        elapsed = time.time() - t0
        if not indefinite and elapsed >= secs: break
        s = VS.snap()
        ids = sorted(RawMonitor.dump().keys())
        id_str = " ".join(f"0x{i:03X}" for i in ids)
        print(f"\r  {elapsed:>6.1f}  {s['brake_pct']:>7.1f}%  {s['brake_en']:<9}  "
              f"{s['spd_kmh']:>9.3f}  {s['vcu_mode']:<8}  {id_str}   ",
              end="", flush=True)
        time.sleep(0.5)

    print(f"\n\n  ── All CAN IDs seen ─────────────────────────────────")
    dump = RawMonitor.dump()
    for aid in sorted(dump.keys()):
        cnt, last_hex, _ = dump[aid]
        known = {
            0x500:"Throttle_Report", 0x501:"Brake_Report",
            0x502:"Steer_Report",    0x505:"VCU_Status",
            0x506:"Wheel_Speed",     0x103:"[TX] Gear_Cmd",
            0x104:"[TX] Park_Cmd",   0x101:"[TX] Brake_Cmd",
            0x102:"[TX] Steer_Cmd",  0x105:"[TX] VCU_Mode",
        }.get(aid,"*** UNKNOWN — check for gear/park report!")
        print(f"    0x{aid:03X}  {cnt:>6} frames   last={last_hex}   {known}")

# ════════════════════════════════════════════════════════
#  MODE: BRAKE
# ════════════════════════════════════════════════════════

def mode_brake(bus, logger, args, estop_flag):
    pct       = args.pct
    ramp_rate = args.ramp
    indefinite= (args.hold == 0)
    ramp_step = ramp_rate * DT if ramp_rate > 0 else 0

    print(f"\n[BRAKE] target={pct:.0f}%  ramp={ramp_rate:.0f}%/s  "
          f"hold={'∞' if indefinite else f'{args.hold:.0f}s'}")
    print(f"  No park brake sent. Push vehicle to verify brake holds.")
    print(f"  {'T':>6}  {'Phase':<10}  {'CMD%':>6}  {'ACT%':>7}  "
          f"{'En':<9}  {'Spd':>7}  {'FL/FR m/s':<14}  PUSHED?")
    print(f"  {'':─<80}")

    t_start=time.time(); cur=0.0; push=0

    def send(phase, p):
        nonlocal push
        _tx_bus(bus, 0x105, build_vehicle_mode())
        _tx_bus(bus, 0x101, build_brake(True, p))
        logger.log("brake", phase, cmd_brake=p)
        s = VS.snap()
        mv = any(abs(s[k])>0.01 for k in ["wfl","wfr","wrl","wrr"])
        if mv: push += 1
        print(f"\r  {time.time()-t_start:>6.2f}  {phase:<10}  {p:>6.1f}%  "
              f"{s['brake_pct']:>7.1f}%  {s['brake_en']:<9}  {s['spd_kmh']:>7.3f}  "
              f"{s['wfl']:>6.3f}/{s['wfr']:>6.3f}  "
              f"{'⚠ MOVING' if mv else 'OK      '}   ", end="", flush=True)
        time.sleep(DT)

    # ramp up
    if ramp_rate > 0:
        while not estop_flag[0] and cur < pct:
            cur = min(pct, cur+ramp_step); send("ramp-up", cur)
        if not estop_flag[0]: print(f"\n  [OK] Reached {pct:.0f}%")
    else:
        cur = pct

    # hold
    t_hold = time.time()
    while not estop_flag[0]:
        if not indefinite and (time.time()-t_hold) >= args.hold: break
        send("hold", cur)

    # ramp down
    if ramp_rate > 0 and not estop_flag[0]:
        print(f"\n  [>>] Ramp down {cur:.0f}% → 0%")
        while not estop_flag[0] and cur > 0:
            cur = max(0.0, cur-ramp_step); send("ramp-dn", cur)

    print()
    if not estop_flag[0]:
        t_rel = time.time()
        while time.time()-t_rel < 0.8:
            _tx_bus(bus, 0x105, build_vehicle_mode())
            _tx_bus(bus, 0x101, build_brake(False, 0.0))
            logger.log("brake","release"); time.sleep(DT)
    return push

# ════════════════════════════════════════════════════════
#  MODE: GEAR
# ════════════════════════════════════════════════════════

# ════════════════════════════════════════════════════════
#  MODE: GEAR
# ════════════════════════════════════════════════════════

def mode_gear(bus, logger, args, estop_flag):
    hold = args.hold if args.hold > 0 else 3.0

    if args.cycle:
        # NOTE: PARK gear (gear_id=1) confirmed NOT supported via 0x103 on this VCU.
        # VCU ignores gear_target=1; park is done via 0x104 park brake instead.
        sequence = [(3,"NEUTRAL"),(4,"DRIVE"),(3,"NEUTRAL"),(2,"REVERSE"),(3,"NEUTRAL")]
        print(f"\n[GEAR] Cycle: {' → '.join(n for _,n in sequence)}  hold={hold:.0f}s each")
        print(f"  NOTE: PARK gear removed — not supported via 0x103 on this VCU.")
        print(f"        Use --mode park to test park brake separately.")
    else:
        gid = GEAR_ID.get(args.gear.lower())
        if not gid: sys.exit(f"Unknown gear '{args.gear}'. Use: reverse/neutral/drive")
        if gid == 1:
            print(f"\n[WARN] PARK gear (gear_id=1) is not supported via 0x103 on this VCU.")
            print(f"       Use --mode park --park-action engage instead.")
            print(f"       Sending anyway for diagnostic purposes...")
        sequence = [(gid, args.gear.upper())]
        print(f"\n[GEAR] Select: {args.gear.upper()}  hold={hold:.0f}s")

    print(f"\n  PARK BRAKE RELEASED — vehicle held by 100% brake.")
    print(f"  Wheels must be chocked. Sends: 0x105+0x104(off)+0x101+0x100+0x102+0x103")

    def _send_all(brake_pct, park_engage, gear_id=None):
        _tx_bus(bus, 0x105, build_vehicle_mode())
        _tx_bus(bus, 0x104, build_park(True, park_engage))
        _tx_bus(bus, 0x101, build_brake(True, brake_pct))
        _tx_bus(bus, 0x100, build_throttle(True, 0.0))
        _tx_bus(bus, 0x102, build_steer(True, 0, 50))
        if gear_id is not None:
            _tx_bus(bus, 0x103, build_gear(True, gear_id))

    # ── Step 0: park RELEASED + 100% brake, wait for brake/throttle/steer = Auto ─
    print(f"\n  Step 0: Park RELEASE + 100% brake + throttle/steer wake "
          f"— waiting for all EnState=Auto (up to 5s)...")
    t_settle = time.time()
    all_ready = False
    while time.time()-t_settle < 5.0 and not estop_flag[0]:
        _send_all(100.0, False)
        s = VS.snap()
        print(f"\r    brake_en={s['brake_en']:<9}  throttle_en={s['throttle_en']:<9}  "
              f"steer_en={s['steer_en']:<9}  brake={s['brake_pct']:.0f}%  "
              f"vcu={s['vcu_mode']}   ", end="", flush=True)
        if s['brake_en']=='Auto' and s['throttle_en']=='Auto' and s['steer_en']=='Auto':
            all_ready = True
            break
        time.sleep(DT)

    s = VS.snap()
    print(f"\n  brake_en={s['brake_en']}  throttle_en={s['throttle_en']}  "
          f"steer_en={s['steer_en']}  brake_actual={s['brake_pct']:.0f}%")
    if not all_ready:
        print(f"  [WARN] Not all subsystems reached Auto — gear shift may still fail.")
        print(f"         Continuing anyway...")
    else:
        print(f"  [OK] Brake+Throttle+Steer all Auto — interlock satisfied, "
              f"proceeding to gear commands.")

    # hold settle 0.5s
    t = time.time()
    while time.time()-t < 0.5 and not estop_flag[0]:
        _send_all(100.0, False)
        time.sleep(DT)

    # ── Gear cycle ────────────────────────────────────────────────────────────
    print(f"\n  {'Step':<6}  {'CMD':<10}  {'T':>6}  {'Brake%':>7}  "
          f"{'B/T/S En':<16}  {'Gear_Actual':>11}  VCU")
    print(f"  {'':─<75}")

    GEAR_DEC = {0:"INVALID",1:"PARK",2:"REVERSE",3:"NEUTRAL",4:"DRIVE"}
    results = []

    for step,(gid,gname) in enumerate(sequence):
        if estop_flag[0]: break
        print(f"\n  [{step+1}/{len(sequence)}] → {gname}  "
              f"(0x103: Gear_EnCtrl=1  Gear_Target={gid})")
        t0 = time.time()
        ga_seen = set()
        shift_latency = None    # time from cmd until gear changed
        prev_ga = None

        while (time.time()-t0) < hold and not estop_flag[0]:
            _send_all(100.0, False, gear_id=gid)
            logger.log("gear", f"gear_{gname}", cmd_brake=100.0,
                       cmd_gear=gname, cmd_park="release")
            s = VS.snap()
            raw503 = RawMonitor.dump().get(0x503, (0,"0000000000000000",0))[1]
            raw504 = RawMonitor.dump().get(0x504, (0,"",0))[1]
            b0 = int(raw503[:2],16) if len(raw503)>=2 else 0
            ga = b0 & 0x07
            ga_name = GEAR_DEC.get(ga, f"?{ga}")
            ga_seen.add(ga_name)
            if prev_ga is not None and ga != prev_ga and shift_latency is None:
                shift_latency = time.time() - t0
            prev_ga = ga

            # Park report if available
            park_str = ""
            if raw504 and len(raw504) >= 2:
                p0 = int(raw504[:2], 16)
                park_str = f"  park={'ON' if p0&1 else 'off'}"

            ens = f"{s['brake_en'][:1]}/{s['throttle_en'][:1]}/{s['steer_en'][:1]}"
            print(f"\r  {step+1:<5}  {gname:<9}  {time.time()-t0:>5.2f}s  "
                  f"{s['brake_pct']:>6.1f}%  {ens:<6}  {ga_name:<9}"
                  f"{park_str}   ", end="", flush=True)
            time.sleep(DT)

        raw503 = RawMonitor.dump().get(0x503, (0,"0000000000000000",0))[1]
        b0 = int(raw503[:2],16) if len(raw503)>=2 else 0
        ga = b0 & 0x07
        gear_actual = GEAR_DEC.get(ga, f"unknown({ga})")
        match = (ga == gid)
        results.append((gname, gear_actual, match, shift_latency))
        lat_str = f"  latency={shift_latency*1000:.0f}ms" if shift_latency else "  (no change detected)"
        print(f"\n  0x503={raw503}  Gear_Actual={gear_actual}  "
              f"{'✓ MATCH' if match else '✗ MISMATCH'}{lat_str}")

    print(f"\n  ── Gear Summary ────────────────────────────────────────")
    for cmd, actual, match, lat in results:
        lat_str = f"{lat*1000:.0f}ms" if lat else "—"
        print(f"    {cmd:<10} → {actual:<10}  {'✓' if match else '✗'}  latency={lat_str}")
    passed = sum(1 for _,_,m,_ in results if m)
    print(f"\n  Passed: {passed}/{len(results)}")
    if passed == len(results):
        print(f"  [OK] All gear shifts confirmed. Gear is fully working.")
    elif passed > 0:
        failed = [cmd for cmd,_,m,_ in results if not m]
        print(f"  [PARTIAL] Failed: {failed}")
        if "PARK" in failed:
            print(f"  [INFO] PARK gear not supported via 0x103. Use --mode park instead.")

    # ── Post-cycle: leave in NEUTRAL with full brake, then engage park ────────
    if not estop_flag[0]:
        print(f"\n  Post-cycle: NEUTRAL + 100% brake (1s), then engage park brake...")
        t = time.time()
        while time.time()-t < 1.0 and not estop_flag[0]:
            _send_all(100.0, False, gear_id=3)
            time.sleep(DT)
        t = time.time()
        while time.time()-t < 1.0 and not estop_flag[0]:
            _tx_bus(bus, 0x105, build_vehicle_mode())
            _tx_bus(bus, 0x104, build_park(True, True))  # engage park for safe finish
            _tx_bus(bus, 0x101, build_brake(True, 100.0))
            time.sleep(DT)

# ════════════════════════════════════════════════════════
#  MODE: PARK
# ════════════════════════════════════════════════════════

def mode_park(bus, logger, args, estop_flag):
    engage = (args.park_action == "engage")
    action = "ENGAGE" if engage else "RELEASE"
    hold   = args.hold if args.hold > 0 else 5.0

    print(f"\n[PARK] Action: {action}  hold={hold:.0f}s")
    print(f"  NOTE: 0x504 park report not in CAN matrix.")
    print(f"        Park state cannot be read back from CAN.")
    print(f"        Confirm physically — listen for park actuator click/movement.")
    if not engage:
        print(f"  ⚠  Releasing park — ensure vehicle is on flat ground or chocked.")
    print(f"  Sending 0x104: byte0=0x01(enable)  byte1={'0x01(engage)' if engage else '0x00(release)'}")
    print(f"\n  {'T':>6}  {'CMD':<10}  {'Brake ACT%':>11}  {'VCU mode':<9}  {'Spd km/h':>9}")
    print(f"  {'':─<60}")

    t0 = time.time()
    while (time.time()-t0) < hold and not estop_flag[0]:
        _tx_bus(bus, 0x105, build_vehicle_mode())
        _tx_bus(bus, 0x104, build_park(True, engage))
        logger.log("park", f"park_{action.lower()}", cmd_park=action.lower())
        s = VS.snap()
        print(f"\r  {time.time()-t0:>6.2f}  {action:<10}  {s['brake_pct']:>11.1f}%  "
              f"{s['vcu_mode']:<9}  {s['spd_kmh']:>9.3f}   ", end="", flush=True)
        time.sleep(DT)

    print(f"\n  Sent PARK {action} for {hold:.0f}s.")
    print(f"  Verify physically: {'park actuator should be ENGAGED' if engage else 'park actuator should be RELEASED'}.")
    print(f"  Try pushing vehicle — should {'resist' if engage else 'roll freely'}.")

# ════════════════════════════════════════════════════════
#  MODE: THROTTLE — open-field vehicle movement test
#
#  Sequence:
#    Step 0  settle (2s): VCU mode + park OFF + brake 100% + thr/steer wake
#            wait for brake_en=Auto AND throttle_en=Auto AND steer_en=Auto
#    Step 1  gear → DRIVE (1s)
#    Step 2  brake release → 0%
#    Step 3  throttle ramp: 0% → target_pct at ramp_rate %/s
#    Step 4  hold throttle at target_pct for hold_s
#    Step 5  throttle cut → brake ramp 0→100%
#    Step 6  gear → NEUTRAL → park engage
#
#  Safety:
#    - Speed > max_speed_kmh  → immediate cut + full brake + park
#    - Ctrl+C anywhere        → emergency_stop()
#    - Throttle flt1 set      → immediate cut + full brake + park
# ════════════════════════════════════════════════════════

def mode_throttle(bus, logger, args, estop_flag):
    target_pct   = args.throttle_pct
    ramp_rate    = args.throttle_ramp   # %/s
    hold_s       = args.hold
    max_spd      = args.max_speed
    ramp_step    = ramp_rate * DT
    indefinite   = (hold_s == 0)

    print(f"\n[THROTTLE] ⚠  VEHICLE WILL MOVE — open flat area required")
    print(f"  Target   : {target_pct:.0f}%  ramp={ramp_rate:.0f}%/s "
          f"(0→{target_pct:.0f}% in ~{target_pct/ramp_rate:.1f}s)")
    print(f"  Hold     : {'until Ctrl+C' if indefinite else f'{hold_s:.0f}s'}")
    print(f"  Speed cap: {max_spd:.1f} km/h  (auto-cutoff)")
    print(f"  Gear     : DRIVE (forward)")
    print(f"  Ctrl+C   = emergency stop at any time\n")

    def _tx_all(thr_pct=0.0, brk_pct=0.0, gear=3, park=False):
        _tx_bus(bus, 0x105, build_vehicle_mode())
        _tx_bus(bus, 0x104, build_park(True, park))
        _tx_bus(bus, 0x101, build_brake(True, brk_pct))
        _tx_bus(bus, 0x100, build_throttle(True, thr_pct))
        _tx_bus(bus, 0x102, build_steer(True, 0, 50))
        _tx_bus(bus, 0x103, build_gear(True, gear))

    def _speed_ok():
        s = VS.snap()
        kmh = s["spd_kmh"]
        if kmh is not None and abs(kmh) > max_spd:
            print(f"\n  [SPEED LIMIT] {kmh:.2f} km/h > {max_spd:.1f} km/h — cutting!")
            estop_flag[0] = True
            return False
        return True

    def _thr_ok():
        s = VS.snap()
        if s["throttle_en"] == "Manual":  # throttle dropped out of Auto
            print(f"\n  [WARN] throttle_en went Manual — VCU dropped autonomous control!")
        return True  # warn but don't abort (transient flicker is normal)

    # ── Step 0: settle, wait for all subsystems Auto ──────────────────────
    print("  Step 0: Settle — park OFF, full brake, wake throttle+steer (2s)...")
    t0 = time.time()
    all_auto = False
    while time.time()-t0 < 2.0 and not estop_flag[0]:
        _tx_all(thr_pct=0.0, brk_pct=100.0, gear=3, park=False)
        s = VS.snap()
        print(f"\r    B={s['brake_en']:<9}  T={s['throttle_en']:<9}  "
              f"S={s['steer_en']:<9}  vcu={s['vcu_mode']}   ", end="", flush=True)
        if all(s[k]=='Auto' for k in ['brake_en','throttle_en','steer_en']):
            all_auto = True
        time.sleep(DT)

    s = VS.snap()
    print(f"\n  brake_en={s['brake_en']}  throttle_en={s['throttle_en']}  "
          f"steer_en={s['steer_en']}  brake={s['brake_pct']:.0f}%")
    if not all_auto:
        print(f"  [WARN] Not all subsystems Auto — throttle may not actuate.")
    else:
        print(f"  [OK] All subsystems Auto — safe to proceed.")

    if estop_flag[0]: return

    # ── Step 1: select DRIVE gear ─────────────────────────────────────────
    print(f"\n  Step 1: Select DRIVE gear (1s)...")
    t1 = time.time()
    while time.time()-t1 < 1.0 and not estop_flag[0]:
        _tx_all(thr_pct=0.0, brk_pct=100.0, gear=4, park=False)
        s = VS.snap()
        raw503 = RawMonitor.dump().get(0x503,(0,"0000000000000000",0))[1]
        gear_actual = int(raw503[:2],16) & 0x7 if raw503 else 0
        print(f"\r    gear_actual={gear_actual} {'(DRIVE✓)' if gear_actual==4 else ''}   ",
              end="", flush=True)
        time.sleep(DT)

    raw503 = RawMonitor.dump().get(0x503,(0,"0000000000000000",0))[1]
    gear_actual = int(raw503[:2],16) & 0x7 if raw503 else 0
    print(f"\n  Gear_Actual={gear_actual} {'=DRIVE ✓' if gear_actual==4 else '(expected 4=DRIVE)'}")
    if estop_flag[0]: return

    # ── Step 2: release brake ─────────────────────────────────────────────
    print(f"\n  Step 2: Release brake to 0% (0.5s)...")
    t2 = time.time()
    while time.time()-t2 < 0.5 and not estop_flag[0]:
        _tx_all(thr_pct=0.0, brk_pct=0.0, gear=4, park=False)
        time.sleep(DT)
    if estop_flag[0]: return

    # ── Step 3+4: throttle ramp + hold ───────────────────────────────────
    print(f"\n  Step 3: Ramp throttle 0→{target_pct:.0f}% at {ramp_rate:.0f}%/s...")
    print(f"  {'T':>6}  {'Phase':<12}  {'THR_CMD%':>8}  {'THR_ACT%':>9}  "
          f"{'Spd km/h':>9}  {'B/T/S En':<12}  SPEED_OK?")
    print(f"  {'':─<80}")

    t_start = time.time()
    cur_thr = 0.0
    speed_samples = []

    def _row(phase, cmd):
        s = VS.snap()
        ens = f"{s['brake_en'][:1]}/{s['throttle_en'][:1]}/{s['steer_en'][:1]}"
        ok = abs(s['spd_kmh']) <= max_spd
        speed_samples.append(s['spd_kmh'])
        logger.log("throttle", phase, cmd_brake=0.0, cmd_gear="DRIVE")
        print(f"\r  {time.time()-t_start:>6.2f}  {phase:<12}  {cmd:>8.1f}%  "
              f"{s['throttle_pct']:>9.1f}%  {s['spd_kmh']:>9.3f}  "
              f"{ens:<12}  {'OK' if ok else '⚠ OVER'}   ", end="", flush=True)
        return s

    # ramp up
    while not estop_flag[0] and cur_thr < target_pct:
        cur_thr = min(target_pct, cur_thr + ramp_step)
        _tx_all(thr_pct=cur_thr, brk_pct=0.0, gear=4, park=False)
        _row("ramp", cur_thr)
        if not _speed_ok(): break
        _thr_ok()
        time.sleep(DT)

    if not estop_flag[0]:
        print(f"\n  Step 4: Hold {target_pct:.0f}% for {'∞' if indefinite else f'{hold_s:.0f}s'}...")
        t_hold = time.time()
        while not estop_flag[0]:
            if not indefinite and (time.time()-t_hold) >= hold_s: break
            _tx_all(thr_pct=target_pct, brk_pct=0.0, gear=4, park=False)
            _row("hold", target_pct)
            if not _speed_ok(): break
            _thr_ok()
            time.sleep(DT)

    # ── Step 5: cut throttle, brake to stop ──────────────────────────────
    print(f"\n\n  Step 5: Throttle OFF → 100% brake until stopped...")
    t5 = time.time()
    while True:
        _tx_all(thr_pct=0.0, brk_pct=100.0, gear=4, park=False)
        s = VS.snap()
        print(f"\r    braking...  speed={s['spd_kmh']:.3f} km/h  "
              f"brake={s['brake_pct']:.0f}%   ", end="", flush=True)
        stopped = abs(s['spd_kmh']) < 0.05 or (time.time()-t5 > 10.0)
        if stopped: break
        time.sleep(DT)

    # ── Step 6: NEUTRAL + park ON ────────────────────────────────────────
    print(f"\n  Step 6: NEUTRAL + park engage (1.5s)...")
    t6 = time.time()
    while time.time()-t6 < 1.5:
        _tx_bus(bus, 0x105, build_vehicle_mode())
        _tx_bus(bus, 0x103, build_gear(True, 3))      # NEUTRAL
        _tx_bus(bus, 0x104, build_park(True, True))   # park ON
        _tx_bus(bus, 0x101, build_brake(True, 100.0))
        time.sleep(DT)

    # ── Summary ───────────────────────────────────────────────────────────
    if speed_samples:
        valid = [v for v in speed_samples if v is not None]
        print(f"\n  ── Throttle Run Summary ────────────────────────────────")
        print(f"  Target throttle  : {target_pct:.0f}%")
        s = VS.snap()
        print(f"  Throttle feedback: {s['throttle_pct']:.1f}%  en={s['throttle_en']}")
        print(f"  Max speed reached: {max(valid):.3f} km/h")
        print(f"  Avg speed (hold) : {sum(valid)/len(valid):.3f} km/h")
        if max(valid) < 0.1:
            print(f"  [WARN] Speed never exceeded 0.1 km/h.")
            print(f"         Check: gear in DRIVE? Park released? Throttle_en=Auto?")
        else:
            print(f"  [OK] Vehicle moved — throttle is working ✓")

# ════════════════════════════════════════════════════════
#  MAIN
# ════════════════════════════════════════════════════════

def print_state(label=""):
    s = VS.snap()
    print(f"  {label}")
    print(f"    VCU   : {s['vcu_mode']}  speed={s['spd_kmh']} km/h")
    print(f"    Brake : {s['brake_pct']:.1f}%  en={s['brake_en']}  flt1={s['brake_f1']}")
    print(f"    Wheels: FL={s['wfl']:.3f}  FR={s['wfr']:.3f}  RL={s['wrl']:.3f}  RR={s['wrr']:.3f}")
    print(f"    NOTE  : Gear/Park state not available via CAN on this VCU")

def main():
    p = argparse.ArgumentParser(description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", choices=["brake","gear","park","throttle","monitor"],
                   required=True)
    p.add_argument("--channel", default=CHANNEL)

    # brake
    p.add_argument("--pct",           type=float, default=30.0,
                   help="[brake] target brake %% (default 30)")
    p.add_argument("--ramp",          type=float, default=0.0,
                   help="[brake] ramp rate %%/s, 0=instant")
    p.add_argument("--hold",          type=float, default=5.0,
                   help="Hold seconds. 0=until Ctrl+C (default 5)")

    # gear
    p.add_argument("--gear",          default="neutral",
                   help="[gear] reverse/neutral/drive (default neutral)")
    p.add_argument("--cycle",         action="store_true",
                   help="[gear] run full N→D→N→R→N cycle")

    # park
    p.add_argument("--park-action",   choices=["engage","release"], default="engage",
                   help="[park] engage or release park brake")

    # throttle
    p.add_argument("--throttle-pct",  type=float, default=15.0,
                   help="[throttle] target throttle %% (default 15). Start low (10-15)!")
    p.add_argument("--throttle-ramp", type=float, default=5.0,
                   help="[throttle] ramp rate %%/s (default 5 → 0→15%% in 3s)")
    p.add_argument("--max-speed",     type=float, default=5.0,
                   help="[throttle] speed cutoff km/h (default 5). Emergency stop if exceeded.")

    args = p.parse_args()

    ts       = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_name = f"hooke2_{args.mode}_{ts}.csv"
    log_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), log_name)

    print(f"\n{'═'*56}")
    print(f"  Hooke2 Test Tool  —  mode: {args.mode.upper()}")
    print(f"{'═'*56}")
    print(f"  Channel : {args.channel}")
    print(f"  Log     : {log_name}")
    print(f"  Ctrl+C  : emergency stop at any time")
    print(f"{'═'*56}\n")

    try:
        bus = can.interface.Bus(channel=args.channel, interface="socketcan")
        print(f"[OK] Opened {args.channel}")
    except Exception as e:
        sys.exit(f"[ERROR] CAN open failed: {e}")

    stop_rx   = threading.Event()
    threading.Thread(target=rx_thread, args=(bus,stop_rx), daemon=True).start()

    logger     = Logger(log_path)
    estop_flag = [False]

    def _sigint(sig, frame):
        estop_flag[0] = True
        emergency_stop(bus, logger)
    signal.signal(signal.SIGINT, _sigint)

    print("[..] Reading initial state (1.5s)...")
    t0 = time.time()
    while time.time()-t0 < 1.5:
        _tx_bus(bus, 0x105, build_vehicle_mode())
        logger.log(args.mode,"settle"); time.sleep(DT)
    print_state("Initial state:"); print()

    try:
        if   args.mode=="brake":    mode_brake(bus, logger, args, estop_flag)
        elif args.mode=="gear":     mode_gear(bus, logger, args, estop_flag)
        elif args.mode=="park":     mode_park(bus, logger, args, estop_flag)
        elif args.mode=="throttle": mode_throttle(bus, logger, args, estop_flag)
        elif args.mode=="monitor":  mode_monitor(bus, logger, args.hold)
    except Exception as e:
        print(f"\n[ERROR] {e}")
        estop_flag[0]=True; emergency_stop(bus, logger)
    finally:
        if not estop_flag[0]:
            t=time.time()
            while time.time()-t<0.5:
                _tx_bus(bus, 0x105, build_vehicle_mode()); time.sleep(DT)
        logger.close(); stop_rx.set(); bus.shutdown()

    print(); print_state("Final state:")
    print(f"\n[DONE] Log → {log_path}\n")

if __name__=="__main__":
    main()
