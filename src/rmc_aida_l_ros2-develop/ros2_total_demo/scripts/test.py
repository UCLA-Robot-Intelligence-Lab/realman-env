#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
aoyi_probe.py

Probe the Aoyi dexterous hand register->joint mapping by writing a small test value
to each register index (0..11) and reading the holding registers back to see which
entries changed.

This uses the same JSON-over-TCP commands your current AoyiHand uses:
  - write_registers
  - read_registers

If your device uses a different "read" JSON key, change read_registers() accordingly.
"""

import socket
import time
import json
import sys
import copy

# ====== CONFIG ======
IP = '169.254.128.18'   # right hand IP in your code
PORT = 8080
MODBUS_ADDR = 1135      # base register address (your code used 1135)
DEVICE_ID = 2
NUM_REGS = 12           # number of registers in the logical array (0..11)
TEST_VALUE = 200        # small test value to write (tune down if motion is too strong)
SLEEP_AFTER_WRITE = 0.25
SOCKET_TIMEOUT = 2.0

# ====== Helper Aoyi client with read support ======
class AoyiProbeClient:
    def __init__(self, ip=IP, port=PORT, timeout=SOCKET_TIMEOUT):
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        print(f"Connecting to {ip}:{port} ...")
        self.sock.connect((ip, port))
        time.sleep(0.1)

    def send_raw(self, payload_json):
        """Send JSON string (ensure CRLF) and attempt to receive a reply."""
        if not payload_json.endswith("\r\n"):
            payload_json = payload_json + "\r\n"
        try:
            # send
            self.sock.send(payload_json.encode('utf-8'))
        except Exception as e:
            print("SEND ERROR:", e)
            raise

        # try to read a reply (non-blocking with timeout)
        try:
            data = self.sock.recv(4096)
            if not data:
                return None
            try:
                s = data.decode('utf-8', errors='ignore').strip()
                # sometimes device sends multiple JSONs; try to parse the first JSON object
                # find first '{' and last '}' to attempt parse
                jstart = s.find('{')
                jend = s.rfind('}')
                if jstart != -1 and jend != -1 and jend > jstart:
                    js = s[jstart:jend+1]
                    return json.loads(js)
                # fallback: return raw string
                return s
            except Exception:
                return s
        except socket.timeout:
            return None
        except Exception as e:
            print("RECV ERROR:", e)
            return None

    def set_modbus_mode(self):
        cmd = {
            "command":"set_modbus_mode",
            "port":1,
            "baudrate":115200,
            "timeout ":2
        }
        return self.send_raw(json.dumps(cmd))

    def write_registers(self, data_list):
        # ensure a 12-length array
        data = list(data_list) + [0] * (NUM_REGS - len(data_list))
        payload = {
            "command":"write_registers",
            "port":1,
            "address":MODBUS_ADDR,
            "num":6,   # vendor uses 6 in your original code; keep it for compatibility
            "data": data,
            "device": DEVICE_ID
        }
        return self.send_raw(json.dumps(payload))

    def read_registers(self, length=NUM_REGS):
        # some firmwares expect 'read_registers' with address/length; adapt if different
        payload = {
            "command":"read_registers",
            "port":1,
            "address":MODBUS_ADDR,
            "length": length,
            "device": DEVICE_ID
        }
        return self.send_raw(json.dumps(payload))

    def close(self):
        try:
            self.sock.close()
        except:
            pass

# ====== Probing logic ======
def pretty_print_list(l):
    return "[" + ", ".join(str(x) for x in l) + "]"

def probe():
    client = None
    try:
        client = AoyiProbeClient()
        time.sleep(0.2)
        print("Setting modbus mode...")
        client.set_modbus_mode()
        time.sleep(0.5)

        print("Reading baseline registers...")
        baseline_resp = client.read_registers(NUM_REGS)
        if baseline_resp is None:
            print("WARNING: no read reply. Proceeding with zeros baseline.")
            baseline = [0]*NUM_REGS
        else:
            # device reply formats vary; try to extract a 'data' or value array
            baseline = None
            if isinstance(baseline_resp, dict):
                # check common keys
                for k in ('data', 'registers', 'values', 'value'):
                    if k in baseline_resp and isinstance(baseline_resp[k], list):
                        baseline = baseline_resp[k][:NUM_REGS]
                        break
                # vendor might return hex strings or single integers; attempt to parse
            if baseline is None:
                # attempt to parse strings inside
                try:
                    # sometimes response returns {"result": [..]}
                    for v in baseline_resp.values():
                        if isinstance(v, list) and len(v) >= NUM_REGS:
                            baseline = v[:NUM_REGS]
                            break
                except Exception:
                    baseline = None

            if baseline is None:
                print("Could not parse baseline from device reply:", baseline_resp)
                baseline = [0]*NUM_REGS

        print("Baseline registers:", pretty_print_list(baseline))

        mapping = {}  # index -> observed delta value

        for idx in range(NUM_REGS):
            print("\n--- Testing register index", idx, " ---")
            # prepare test array = baseline but with change at idx
            test_array = baseline.copy()
            test_array[idx] = (test_array[idx] or 0) + TEST_VALUE
            print("Writing:", pretty_print_list(test_array))
            resp = client.write_registers(test_array)
            time.sleep(SLEEP_AFTER_WRITE)

            # read back registers
            read_resp = client.read_registers(NUM_REGS)
            read_vals = None
            if isinstance(read_resp, dict):
                for k in ('data','registers','values'):
                    if k in read_resp and isinstance(read_resp[k], list):
                        read_vals = read_resp[k][:NUM_REGS]
                        break
                # try other heuristics
                if read_vals is None:
                    # maybe entire dict is the list
                    for v in read_resp.values():
                        if isinstance(v, list) and len(v) >= NUM_REGS:
                            read_vals = v[:NUM_REGS]
                            break
            if read_vals is None:
                # try to parse string replies
                if isinstance(read_resp, str):
                    # attempt to find digits
                    import re
                    nums = re.findall(r"-?\d+", read_resp)
                    if len(nums) >= NUM_REGS:
                        read_vals = [int(n) for n in nums[:NUM_REGS]]

            if read_vals is None:
                print("READ FAILED or could not parse reply:", read_resp)
                # still attempt to restore baseline
                print("Restoring baseline registers...")
                client.write_registers(baseline)
                time.sleep(0.1)
                mapping[idx] = None
                continue

            print("Read back:", pretty_print_list(read_vals))
            # compute deltas
            deltas = [read_vals[i] - (baseline[i] or 0) for i in range(NUM_REGS)]
            print("Deltas :", pretty_print_list(deltas))
            mapping[idx] = deltas

            # restore baseline
            client.write_registers(baseline)
            time.sleep(0.05)

        print("\n=== PROBE COMPLETE ===")
        for k in range(NUM_REGS):
            print(f"Index {k:2d} deltas: {mapping[k]}")
        print("\nInterpretation: for each test index i, look for which positions show a significant nonzero delta.")
        print("If test at index i gives a delta at position j, then logical register j is affected by writing to index i (or the device uses different ordering).")
        print("Common mapping guess (vendor-typical):")
        print("  registers 0-1: index finger joints")
        print("  registers 2-3: middle finger joints")
        print("  registers 4-5: ring finger joints")
        print("  registers 6-7: pinky finger joints")
        print("  registers 8-9: thumb bends (two joints)")
        print("  register 10 : thumb rotation")
        print("  register 11 : mode / unused / grip state")
        print("\nUse the printed deltas to determine the exact mapping for your hand.")
        return mapping

    except Exception as e:
        print("ERROR during probe:", e)
        raise
    finally:
        if client:
            client.close()

if __name__ == '__main__':
    print("** RUN THIS ON THE ROBOT HOST THAT HAS THE Aoyi HAND CONNECTED **")
    print("Ensure the area is clear and use small TEST_VALUE if the hand moves too strongly.")
    mapping = probe()
    # optionally write mapping to file
    try:
        import json
        with open("aoyi_register_mapping.json", "w") as f:
            json.dump(mapping, f, indent=2)
        print("Saved mapping to aoyi_register_mapping.json")
    except Exception:
        pass
