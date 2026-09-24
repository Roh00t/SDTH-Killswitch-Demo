"""Does this network carry CoT multicast? Answer it in 30 seconds, not on Sunday.

Venue wifi very often blocks multicast or enables AP client isolation, and the
failure is silent: the bridge broadcasts happily, WinTAK shows an empty map, and
you lose the demo wondering which of six things is wrong.

Run RECEIVER on the WinTAK machine and SENDER on the bridge machine. If the
receiver sees nothing, stop debugging and switch to `--loopback` for the demo.

    # same machine (proves the stack, not the network)
    python -m tools.multicast_test --both

    # two machines (proves the VENUE)
    python -m tools.multicast_test --recv          # WinTAK laptop
    python -m tools.multicast_test --send          # bridge laptop

    # after the verdict
    python -m tools.multicast_test --recv --loopback
"""
from __future__ import annotations

import argparse
import socket
import struct
import sys
import threading
import time
from datetime import datetime, timezone

from helper.comms.cot import (
    COT_FRIENDLY_GROUND,
    TAK_LOOPBACK_HOST,
    TAK_LOOPBACK_PORT,
    TAK_MULTICAST_GROUP,
    TAK_MULTICAST_PORT,
    CotEvent,
    build_cot,
    parse_cot,
)


def local_ips() -> list:
    """Every non-loopback IPv4 on this host — candidates for --interface."""
    found = []
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ip = info[4][0]
            if not ip.startswith("127.") and ip not in found:
                found.append(ip)
    except socket.gaierror:
        pass
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(("8.8.8.8", 80))          # no traffic sent; reveals default route
        ip = probe.getsockname()[0]
        probe.close()
        if ip not in found:
            found.insert(0, ip)
    except OSError:
        pass
    return found


def receiver(loopback: bool, interface: str, seconds: float) -> int:
    host, port = ((TAK_LOOPBACK_HOST, TAK_LOOPBACK_PORT) if loopback
                  else ("", TAK_MULTICAST_PORT))
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.bind((host, port))
    except OSError as exc:
        print(f"  Could not bind {host or '0.0.0.0'}:{port}: {exc}")
        print("  Another listener (WinTAK itself?) may already hold this port.")
        return 2

    if not loopback:
        # Join the group. On a multi-homed host the interface matters: join on
        # the wrong adapter and you will never see a packet.
        mreq = struct.pack("4s4s", socket.inet_aton(TAK_MULTICAST_GROUP),
                           socket.inet_aton(interface or "0.0.0.0"))
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        print(f"  Joined {TAK_MULTICAST_GROUP}:{TAK_MULTICAST_PORT} "
              f"on interface {interface or 'DEFAULT'}")
    else:
        print(f"  Listening {TAK_LOOPBACK_HOST}:{TAK_LOOPBACK_PORT} (loopback unicast)")

    sock.settimeout(0.5)
    deadline = time.monotonic() + seconds
    good = bad = 0
    senders = set()
    while time.monotonic() < deadline:
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        senders.add(addr[0])
        try:
            ev = parse_cot(data)
            good += 1
            if good <= 3:
                print(f"  [{good}] {ev.callsign} {ev.cot_type} "
                      f"@ {ev.lat:.5f},{ev.lon:.5f}  from {addr[0]}")
        except Exception:
            bad += 1
    sock.close()

    print(f"\n  valid CoT: {good}   unparseable: {bad}   senders: {sorted(senders) or 'none'}")
    if good:
        if loopback:
            print("\n  VERDICT: LOOPBACK works. This proves the stack, NOT the venue.")
            print("  Run --recv/--send on two machines to test the actual network.")
        else:
            print("\n  VERDICT: multicast WORKS.")
            if interface:
                print(f"  It required an explicit interface. Launch the bridge with:")
                print(f"      python -m tools.c2_bridge --multicast-if {interface}")
            else:
                print("  Default interface was sufficient. Plain bridge mode is fine.")
        return 0

    print("\n  VERDICT: NOTHING RECEIVED.")
    print("  Causes in observed-likelihood order:")
    if not interface:
        ips = local_ips()
        print("    1. WRONG ADAPTER (most common, and silent). Retry with:")
        for ip in ips[:3]:
            print(f"         python -m tools.multicast_test --both --interface {ip}")
        print("       Verified on a dev Mac: default join received 0 frames;")
        print("       binding the adapter explicitly received all of them.")
    else:
        print(f"    1. Adapter {interface} is bound but carries no traffic — try another.")
    print("    2. Venue wifi blocks multicast, or AP client isolation is on.")
    print("    3. Windows Firewall — run tools\\win_firewall_cot.ps1 as Administrator.")
    print("    4. Network profile is Public (Windows blocks inbound despite the rule).")
    print("\n  DECISION RULE: if step 1 does not fix it in five minutes, stop.")
    print("      python -m tools.c2_bridge --loopback")
    print(f"      and point WinTAK at {TAK_LOOPBACK_HOST}:{TAK_LOOPBACK_PORT}, one laptop.")
    return 1


def sender(loopback: bool, interface: str, seconds: float) -> int:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    if loopback:
        dest = (TAK_LOOPBACK_HOST, TAK_LOOPBACK_PORT)
    else:
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 2)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_LOOP, 1)
        if interface:
            sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_IF,
                            socket.inet_aton(interface))
        dest = (TAK_MULTICAST_GROUP, TAK_MULTICAST_PORT)

    print(f"  Sending to {dest[0]}:{dest[1]}"
          + (f" via {interface}" if interface else "") + f" for {seconds:.0f}s")
    sent = 0
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        ev = CotEvent(uid="MCAST_TEST_01", callsign="MULTICAST-TEST",
                      cot_type=COT_FRIENDLY_GROUND, lat=1.3483, lon=103.6831,
                      hae=20.0, stale_seconds=6.0)
        try:
            sock.sendto(build_cot(ev), dest)
            sent += 1
        except OSError as exc:
            print(f"  send failed: {exc}")
            break
        time.sleep(0.5)
    sock.close()
    print(f"  sent {sent} frames (this proves nothing on its own — "
          f"check the RECEIVER)")
    return 0


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--recv", action="store_true", help="Listen for CoT")
    p.add_argument("--send", action="store_true", help="Transmit test CoT")
    p.add_argument("--both", action="store_true", help="Both, same machine")
    p.add_argument("--loopback", action="store_true",
                   help=f"Use {TAK_LOOPBACK_HOST}:{TAK_LOOPBACK_PORT} unicast")
    p.add_argument("--interface", default="", help="Local IPv4 to bind (multi-homed hosts)")
    p.add_argument("--seconds", type=float, default=12.0)
    args = p.parse_args()

    print(f"\nCoT multicast check  —  {datetime.now(timezone.utc):%H:%M:%SZ}")
    print(f"  target : {TAK_LOOPBACK_HOST}:{TAK_LOOPBACK_PORT} (loopback)"
          if args.loopback else
          f"  target : {TAK_MULTICAST_GROUP}:{TAK_MULTICAST_PORT} (multicast)")
    ips = local_ips()
    print(f"  local  : {', '.join(ips) if ips else 'none found'}")
    if len(ips) > 1 and not args.interface and not args.loopback:
        print(f"  NOTE   : {len(ips)} adapters. If this fails, retry with "
              f"--interface {ips[0]}")
    print()

    if args.both:
        t = threading.Thread(target=sender, args=(args.loopback, args.interface,
                                                  args.seconds - 1), daemon=True)
        t.start()
        return receiver(args.loopback, args.interface, args.seconds)
    if args.send:
        return sender(args.loopback, args.interface, args.seconds)
    if args.recv:
        return receiver(args.loopback, args.interface, args.seconds)
    p.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
