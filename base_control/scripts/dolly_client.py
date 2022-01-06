#!/usr/bin/env python3

import socket
import sys, time


def main():
    target_host = '192.168.1.210'
    target_port = 9999

    try:
        client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    except socket.error:
        print('Could not create a socket')
        time.sleep(1)
        sys.exit()

    try:
        client.connect((target_host, target_port))
    except socket.error:
        print('Could not connect to server')
        time.sleep(1)
        sys.exit()

    online = True
    while online:
        data = input()
        client.sendall(data.encode())
        while True:
            message = client.recv(4096)
            if 'q^' in message.decode():
                client.close()
                online = False
                break

            print('[+] Received: ' + message.decode())
            break  # stop receiving

# start client
main()