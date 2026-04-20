#!/usr/bin/env python3
"""Simple UDP log listener for ESP32 log stream."""

import argparse
import socket
import sys
from datetime import datetime


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Listen to UDP logs and print them to stdout")
    parser.add_argument("--port", type=int, default=14514, help="UDP port to listen on (default: 14514)")
    parser.add_argument("--host", default="0.0.0.0", help="Bind address (default: 0.0.0.0)")
    parser.add_argument(
        "--timestamp",
        action="store_true",
        help="Prefix each received packet with local timestamp and sender address",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        sock.bind((args.host, args.port))
    except OSError as exc:
        print(f"Failed to bind UDP listener on {args.host}:{args.port}: {exc}", file=sys.stderr)
        return 1

    print(f"Listening for UDP logs on {args.host}:{args.port}")
    print("Press Ctrl+C to stop")

    try:
        while True:
            data, addr = sock.recvfrom(65535)
            text = data.decode("utf-8", errors="replace")
            if args.timestamp:
                ts = datetime.now().strftime("%H:%M:%S.%f")[:-3]
                print(f"[{ts}] {addr[0]}:{addr[1]} {text}", end="")
            else:
                print(text, end="")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\nStopped UDP listener")
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
