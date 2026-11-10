#!/usr/bin/env python3
import dronecan, time, threading, socket

# get command line arguments
from argparse import ArgumentParser

def main():
    parser = ArgumentParser(description='mavlink DroneCAN tunnel')
    parser.add_argument("--node-id", default=100, type=int, help="CAN node ID")
    parser.add_argument("--debug", action='store_true', help="enable debug")
    parser.add_argument("--tcp-host", type=str, default="localhost", help="tcp server host")
    parser.add_argument("--tcp-port", type=int, default=5790, help="tcp server port")
    parser.add_argument("--rate", type=float, default=10, help="broadcast rate Hz")
    parser.add_argument("uri", default=None, type=str, help="CAN URI")
    args = parser.parse_args()

    # Create a TCP/IP socket
    tcp_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # Bind the socket to the address
    server_address = (args.tcp_host, args.tcp_port)
    tcp_sock.bind(server_address)

    # Listen for incoming connections
    tcp_sock.listen(1)
    tcp_sock.settimeout(0.2)
    connection = None
    connection_lock = threading.Lock()
    stopping = threading.Event()

    def handle_Targetted(msg):
        '''handle Targetted message'''
        if args.debug:
            # display the message on the console in human readable format
            print(dronecan.to_yaml(msg))

        if msg.message.target_node == args.node_id:
            if msg.message.buffer:
                with connection_lock:
                    client = connection
                if client is not None:
                    try:
                        client.sendall(bytes(msg.message.buffer))
                    except ConnectionError:
                        pass  # The peer may have disconnected during the send.
                    except OSError:
                        with connection_lock:
                            if connection is client and not stopping.is_set():
                                raise

    def publish_NodeInfo():
        msg = dronecan.uavcan.protocol.NodeStatus()
        msg.uptime_sec = int(time.time())
        msg.health = msg.HEALTH_OK
        msg.mode = msg.MODE_OPERATIONAL
        msg.submode = 0
        msg.vendor_specific_status_code = 0
        node.broadcast(msg)
        if args.debug:
            # display the message on the console in human readable format
            print(dronecan.to_yaml(msg))

    def publish_MavlinkMsg(mav_pkt):
        '''send Tunnel message'''
        buffer = list(mav_pkt)
        chunks = [buffer[i:i + 120] for i in range(0, len(buffer), 120)]
        for chunk in chunks:
            msg = dronecan.uavcan.tunnel.Targetted()
            msg.serial_id = 1
            msg.target_node = 10
            msg.protocol.protocol = 1 # MAVLink2
            msg.baudrate = 115200
            msg.options = msg.OPTION_LOCK_PORT
            msg.buffer = chunk
            node.broadcast(msg)
            if args.debug:
                # display the message on the console in human readable format
                print(dronecan.to_yaml(msg))

    def tcp_server_loop():
        nonlocal connection
        print(f'Waiting for GCS Connection on tcp:{args.tcp_host}:{args.tcp_port}...')
        while not stopping.is_set():
            try:
                client, _ = tcp_sock.accept()
            except socket.timeout:
                continue

            with client:
                with connection_lock:
                    if stopping.is_set():
                        return
                    connection = client
                print("GCS connected")
                try:
                    while not stopping.is_set():
                        try:
                            data = client.recv(1024)
                        except ConnectionError:
                            break
                        except OSError:
                            if not stopping.is_set():
                                raise
                            break
                        if not data:
                            break
                        publish_MavlinkMsg(data)
                finally:
                    with connection_lock:
                        connection = None

    # Initializing a DroneCAN node instance.
    node = dronecan.make_node(args.uri, node_id=args.node_id, bitrate=1000000)

    # Initializing a node monitor, so we can see what nodes are online
    node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

    node.periodic(1.0/args.rate, publish_NodeInfo)

    node.add_handler(dronecan.uavcan.tunnel.Targetted, handle_Targetted)

    tcp_server_loop_thread = threading.Thread(target=tcp_server_loop)
    tcp_server_loop_thread.start()

    # Running the node until the application is terminated or until first error.
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        stopping.set()
        with connection_lock:
            if connection is not None:
                # Wake the worker if it is waiting for data from a client.
                try:
                    connection.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass  # The peer may have disconnected already.
        tcp_server_loop_thread.join()
        tcp_sock.close()
        node.close()


if __name__ == '__main__':
    main()
