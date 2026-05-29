#!/usr/bin/env python3
"""
Fetch all DroneCAN parameters from a target node, using the same
pipelined GetSet logic that the dronecan_gui_tool node-properties
panel does (gui_tool/dronecan_gui_tool/widgets/node_properties.py).

The pipeline keeps PARAM_FETCH_PIPELINE_DEPTH=5 GetSet requests in
flight at a time, retries each up to PARAM_FETCH_MAX_RETRIES=5 with a
PARAM_FETCH_RETRY_DELAY=0.1 s backoff, and stops when it sees the
end-of-params marker (a response whose name is empty).

Designed for diagnosing param-fetch slowness / loss: it prints
per-index timing, retries, and overall stats so we can see exactly
where the target is dropping or stalling.

Usage:
    dronecan_paramfetch.py --bus mcast:0 --target 116
    dronecan_paramfetch.py --bus slcan:/dev/ttyACM0 --target 115 --verbose
"""
import argparse
import sys
import time
from functools import partial

import dronecan


# These constants are copied verbatim from
# gui_tool/dronecan_gui_tool/widgets/node_properties.py — keep in sync.
REQUEST_PRIORITY = 30
PARAM_FETCH_PIPELINE_DEPTH = 5
PARAM_FETCH_MAX_RETRIES = 5
PARAM_FETCH_RETRY_DELAY = 0.1


class ParamFetcher:
    """
    Mirror of NodePropertiesWindow's param-fetch state machine. The only
    differences from gui_tool are (a) no Qt — we drive node.spin() from
    a plain loop — and (b) we record per-index stats for diagnosis.
    """

    def __init__(self, node, target_node_id, verbose=False):
        self._node = node
        self._target_node_id = target_node_id
        self._verbose = verbose

        self._fetch_in_progress = False
        self._fetch_generation = 0
        self._next_index_to_send = 0
        self._end_index = None
        self._pending = {}       # index -> retry_count
        self._received = {}      # index -> response (waiting to flush in order)
        self._params = []        # ordered list of fetched responses

        # diagnostic state
        self._send_times = {}        # index -> last send wall time
        self._first_send_times = {}  # index -> first send wall time
        self._rtt = {}               # index -> round-trip time of successful response
        self._retries = {}           # index -> total retry count
        self._timeline = []          # (t_relative, event_str)
        self._t0 = None

    # ------------------------------------------------------------------
    # Internal helpers — exact copies of NodePropertiesWindow's, only
    # adapted to use stdout instead of Qt status-bar messages.
    # ------------------------------------------------------------------

    def _log(self, msg):
        t = time.monotonic() - self._t0
        self._timeline.append((t, msg))
        if self._verbose:
            print(f"  [{t:7.3f}s] {msg}")

    def _send_param_request(self, index, retry_count, generation):
        if generation != self._fetch_generation:
            return False  # fetch aborted/restarted
        self._pending[index] = retry_count
        self._send_times[index] = time.monotonic()
        if index not in self._first_send_times:
            self._first_send_times[index] = self._send_times[index]
        try:
            self._node.request(dronecan.uavcan.protocol.param.GetSet.Request(index=index),
                               self._target_node_id,
                               partial(self._on_fetch_response, index, generation),
                               priority=REQUEST_PRIORITY)
            self._log(f"send idx={index} retry={retry_count}")
            return True
        except Exception as ex:
            self._log(f"send error idx={index}: {ex!r}")
            self._pending.pop(index, None)
            self._abort_fetch()
            return False

    def _send_next_param_request(self):
        if self._end_index is not None and self._next_index_to_send >= self._end_index:
            return False
        index = self._next_index_to_send
        self._next_index_to_send += 1
        return self._send_param_request(index, 0, self._fetch_generation)

    def _flush_received(self):
        next_idx = len(self._params)
        while next_idx in self._received:
            resp = self._received.pop(next_idx)
            self._params.append((next_idx, resp))
            next_idx += 1

    def _try_finish_fetch(self):
        if not self._fetch_in_progress:
            return
        if self._end_index is not None and not self._pending:
            self._flush_received()
            self._fetch_in_progress = False
            self._log(f"finished: {len(self._params)} params, end_index={self._end_index}")

    def _abort_fetch(self):
        if not self._fetch_in_progress:
            return
        self._fetch_in_progress = False
        self._fetch_generation += 1

    def _on_fetch_response(self, index, generation, e):
        if generation != self._fetch_generation:
            return  # stale response from a prior fetch

        if e is None:
            # Timeout. If we know end_index and this index is past it, drop silently.
            if self._end_index is not None and index >= self._end_index:
                self._pending.pop(index, None)
                self._try_finish_fetch()
                return
            retry_count = self._pending.get(index, 0) + 1
            self._retries[index] = retry_count
            if retry_count <= PARAM_FETCH_MAX_RETRIES:
                self._pending[index] = retry_count
                self._log(f"TIMEOUT idx={index} retry {retry_count}/{PARAM_FETCH_MAX_RETRIES}")
                self._node.defer(PARAM_FETCH_RETRY_DELAY,
                                 partial(self._send_param_request, index, retry_count, generation))
            else:
                self._log(f"GIVEN UP idx={index} after {PARAM_FETCH_MAX_RETRIES} retries")
                self._pending.pop(index, None)
                self._abort_fetch()
            return

        # Got a response
        send_t = self._send_times.get(index)
        rtt = (time.monotonic() - send_t) if send_t is not None else None
        self._rtt[index] = rtt
        self._pending.pop(index, None)

        if len(e.response.name) == 0:
            # End-of-params marker
            if self._end_index is None or index < self._end_index:
                self._end_index = index
            self._log(f"end-marker idx={index} rtt={rtt*1000:.1f}ms")
            self._try_finish_fetch()
            return

        name = bytes(e.response.name).decode('latin-1', errors='replace')
        self._received[index] = e.response
        self._flush_received()
        self._log(f"recv idx={index} '{name}' rtt={rtt*1000:.1f}ms pending={len(self._pending)}")
        self._send_next_param_request()
        self._try_finish_fetch()

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self, overall_timeout=30.0):
        """Mirror of _do_reload(). Returns True if fetch completed cleanly."""
        self._t0 = time.monotonic()
        self._fetch_generation += 1
        self._fetch_in_progress = True
        self._next_index_to_send = 0
        self._end_index = None
        self._pending = {}
        self._received = {}
        self._params = []

        self._log(f"start: pipeline depth={PARAM_FETCH_PIPELINE_DEPTH}")
        for _ in range(PARAM_FETCH_PIPELINE_DEPTH):
            if not self._send_next_param_request():
                break

        deadline = time.monotonic() + overall_timeout
        while self._fetch_in_progress and time.monotonic() < deadline:
            self._node.spin(0.05)

        if self._fetch_in_progress:
            self._log("ABORTING: overall timeout reached")
            self._abort_fetch()
            return False
        return True

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def stats(self):
        """Return a dict of summary stats for the last fetch."""
        total = self._t0 and (
            self._timeline[-1][0] if self._timeline else 0
        )
        retried = {i: r for i, r in self._retries.items() if r > 0}
        rtts = [t for t in self._rtt.values() if t is not None]
        rtts.sort()
        def pct(p):
            if not rtts:
                return None
            k = max(0, min(len(rtts) - 1, int(round(p * (len(rtts) - 1)))))
            return rtts[k]
        return {
            'total_seconds': total,
            'params_received': len(self._params),
            'end_index': self._end_index,
            'indices_retried': retried,
            'indices_given_up': [i for i, r in self._retries.items() if r > PARAM_FETCH_MAX_RETRIES],
            'rtt_min_ms': pct(0) and pct(0) * 1000,
            'rtt_p50_ms': pct(0.5) and pct(0.5) * 1000,
            'rtt_p90_ms': pct(0.9) and pct(0.9) * 1000,
            'rtt_max_ms': pct(1) and pct(1) * 1000,
        }

    def print_param_table(self):
        for idx, resp in self._params:
            name = bytes(resp.name).decode('latin-1', errors='replace')
            tag = getattr(resp.value, '_union_field', '?')
            if tag == 'integer_value':
                val = f"int={resp.value.integer_value}"
            elif tag == 'boolean_value':
                val = f"bool={bool(resp.value.boolean_value)}"
            elif tag == 'string_value':
                val = f"str(len={len(resp.value.string_value)})"
            else:
                val = f"?{tag}"
            print(f"  {idx:3d}  {name:<26s}  {val}")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--bus', default='mcast:0',
                   help="DroneCAN bus (mcast:0, slcan:/dev/ttyACM0, etc.)")
    p.add_argument('--target', type=int, required=True,
                   help="Target node ID to fetch from")
    p.add_argument('--node-id', type=int, default=100,
                   help="Local node ID to use")
    p.add_argument('--bitrate', type=int, default=1000000,
                   help="CAN bitrate (default 1Mbit)")
    p.add_argument('--timeout', type=float, default=30.0,
                   help="Overall fetch timeout (s)")
    p.add_argument('--verbose', '-v', action='store_true',
                   help="Print each send/recv/timeout event with timestamp")
    p.add_argument('--repeat', type=int, default=1,
                   help="Run the fetch this many times back-to-back")
    p.add_argument('--list', action='store_true',
                   help="Print the fetched parameter table at the end")
    args = p.parse_args()

    print(f"connecting to {args.bus} as node {args.node_id} ...")
    node = dronecan.make_node(args.bus, node_id=args.node_id, bitrate=args.bitrate)

    # Brief settle window so node-status broadcasts establish the
    # peer before we start firing requests.
    end = time.monotonic() + 1.5
    while time.monotonic() < end:
        node.spin(0.05)

    overall_ok = True
    for run_i in range(args.repeat):
        if args.repeat > 1:
            print(f"\n=== fetch run {run_i + 1}/{args.repeat} ===")
        fetcher = ParamFetcher(node, args.target, verbose=args.verbose)
        ok = fetcher.run(overall_timeout=args.timeout)
        stats = fetcher.stats()
        print(f"  total_time:        {stats['total_seconds']:.3f} s")
        print(f"  params_received:   {stats['params_received']}")
        print(f"  end_index:         {stats['end_index']}")
        print(f"  rtt p50/p90/max:   {stats['rtt_p50_ms']:.1f} / "
              f"{stats['rtt_p90_ms']:.1f} / {stats['rtt_max_ms']:.1f} ms"
              if stats['rtt_p50_ms'] is not None else "  rtt: <no samples>")
        if stats['indices_retried']:
            retried = ', '.join(f"{i}({r})" for i, r in
                                sorted(stats['indices_retried'].items()))
            print(f"  indices retried:   {retried}")
        if stats['indices_given_up']:
            print(f"  indices GIVEN UP:  {stats['indices_given_up']}")
        if not ok:
            overall_ok = False
        if args.list and ok:
            print("  --- params ---")
            fetcher.print_param_table()

    sys.exit(0 if overall_ok else 1)


if __name__ == '__main__':
    main()
