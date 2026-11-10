import json
import os
from pathlib import Path
from queue import Queue
import runpy
import signal
import socket
import struct
import subprocess
import sys
from threading import Thread
from types import SimpleNamespace
import unittest


EXAMPLE = Path(__file__).resolve().parents[1] / 'examples' / 'mavlink_tunnel.py'


class TunnelMessage:
    OPTION_LOCK_PORT = 1

    def __init__(self):
        self.protocol = SimpleNamespace()


class FakeNode:
    def periodic(self, *_args):
        pass

    def add_handler(self, _message_type, callback):
        self.handle = callback

    def broadcast(self, message):
        print(json.dumps({
            'payload': bytes(message.buffer).decode('ascii'),
            'target': message.target_node,
            'protocol': message.protocol.protocol,
        }))

    def spin(self):
        print('node ready', file=sys.stderr)
        for line in sys.stdin:
            self.handle(SimpleNamespace(message=SimpleNamespace(
                target_node=100, buffer=line.rstrip('\n').encode('ascii'),
            )))

    def close(self):
        print('node closed')


@unittest.skipUnless(os.name == 'posix', 'requires sending SIGINT to a subprocess')
class MavlinkTunnelTests(unittest.TestCase):
    def start_tunnel(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            port = reservation.getsockname()[1]
        child = subprocess.Popen(
            [sys.executable, '-u', __file__, '--tunnel-child', str(port)],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
        )

        lines = Queue()

        def read_output(stream):
            for line in stream:
                lines.put(line.rstrip())

        readers = [Thread(target=read_output, args=(stream,), daemon=True) for stream in (child.stdout, child.stderr)]
        for reader in readers:
            reader.start()

        def clean_up():
            if child.poll() is None:
                child.kill()
            child.wait(timeout=5)
            for reader in readers:
                reader.join(timeout=5)
            child.stdin.close()
            child.stdout.close()
            child.stderr.close()

        self.addCleanup(clean_up)
        self.assertCountEqual([lines.get(timeout=5), lines.get(timeout=5)], [
            'node ready', f'Waiting for GCS Connection on tcp:127.0.0.1:{port}...',
        ])
        return child, lines, ('127.0.0.1', port)

    def check_forwarding(self, child, lines, client, payload):
        self.assertEqual(lines.get(timeout=5), 'GCS connected')
        client.sendall(payload.encode('ascii'))
        self.assertEqual(json.loads(lines.get(timeout=5)), {'payload': payload, 'target': 10, 'protocol': 1})
        child.stdin.write(payload + '\n')
        child.stdin.flush()
        with client.makefile('rb') as response:
            self.assertEqual(response.read(len(payload)), payload.encode('ascii'))

    def interrupt(self, child, lines):
        child.send_signal(signal.SIGINT)
        self.assertEqual(lines.get(timeout=5), 'node closed')
        self.assertEqual(child.wait(timeout=5), 0)

    def test_interrupt_while_listening(self):
        child, lines, _ = self.start_tunnel()
        self.interrupt(child, lines)

    def test_reconnect_and_interrupt_active_client(self):
        child, lines, address = self.start_tunnel()
        with socket.create_connection(address, timeout=5) as first:
            self.check_forwarding(child, lines, first, 'first')
            first.shutdown(socket.SHUT_WR)
            self.assertEqual(first.recv(1), b'')
        with socket.create_connection(address, timeout=5) as second:
            self.check_forwarding(child, lines, second, 'second')
            self.interrupt(child, lines)
            self.assertEqual(second.recv(1), b'')

    def test_reconnect_after_client_reset(self):
        child, lines, address = self.start_tunnel()
        with socket.create_connection(address, timeout=5) as first:
            self.check_forwarding(child, lines, first, 'first')
            first.setsockopt(socket.SOL_SOCKET, socket.SO_LINGER, struct.pack('ii', 1, 0))
        with socket.create_connection(address, timeout=5) as second:
            self.check_forwarding(child, lines, second, 'second')
            self.interrupt(child, lines)


if __name__ == '__main__':
    if len(sys.argv) > 1 and sys.argv[1] == '--tunnel-child':
        sys.modules['dronecan'] = SimpleNamespace(
            make_node=lambda *_args, **_kwargs: FakeNode(),
            app=SimpleNamespace(node_monitor=SimpleNamespace(NodeMonitor=lambda _node: None)),
            uavcan=SimpleNamespace(tunnel=SimpleNamespace(Targetted=TunnelMessage)),
        )
        sys.argv = [str(EXAMPLE), '--tcp-host', '127.0.0.1', '--tcp-port', sys.argv[2], 'mock-can']
        runpy.run_path(str(EXAMPLE), run_name='__main__')
    else:
        unittest.main()
