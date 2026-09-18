#!/usr/bin/env python3
"""Serve one directory on an ephemeral loopback port for browser probes."""

import argparse
import functools
import http.server
import json
import os
import pathlib
import signal


class QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("root")
    parser.add_argument("ready_file")
    args = parser.parse_args()
    root = pathlib.Path(args.root).resolve(strict=True)
    ready = pathlib.Path(args.ready_file)
    if ready.exists():
        parser.error(f"ready file already exists: {ready}")
    handler = functools.partial(QuietHandler, directory=str(root))
    server = http.server.ThreadingHTTPServer(("127.0.0.1", 0), handler)
    server.daemon_threads = True
    url = f"http://127.0.0.1:{server.server_address[1]}"
    ready.write_text(json.dumps({"pid": os.getpid(), "url": url}) + "\n")

    def stop(_signum, _frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, stop)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
