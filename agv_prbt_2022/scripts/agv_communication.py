#!/usr/bin/env python3
from struct import pack, unpack
import socket
import time


def socketConnect():
    # AGV车管IP地址及端口
    agv_ip = "192.168.251.50"
    agv_port = 9998

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.connect((agv_ip, agv_port))
    except socket.error as e:
        print("Connection error: %s" % e)
    return sock


def agv_communication_deploy(task_id, value, data_length=4, task_type=0, result=0):
    sock_local = socketConnect()

    # reseive buffer size
    buffer_size = 20

    # format for byte transfer
    msg_fmt = "!5I"

    # pack message
    message = pack(msg_fmt, data_length, task_id, task_type, value, result)

    while True:
        try:
            sock_local.send(message)
            data = sock_local.recv(buffer_size)
            receive_frame = unpack(msg_fmt, data)
            TaskId = receive_frame[1]
            Result = receive_frame[4]
            sock_local.close()
            break
        except socket.error:
            print("Socket connection error, reconnecting...")
            time.sleep(3)
            sock_local = socketConnect()
        except:
            print("Other Error")
            time.sleep(3)
        time.sleep(1)

    return TaskId, Result


def agv_communication_lookup(task_id, value, data_length=4, task_type=1, result=0):
    sock_local = socketConnect()

    # reseive buffer size
    buffer_size = 20

    # format for byte transfer
    msg_fmt = "!5I"

    # pack message
    message = pack(msg_fmt, data_length, task_id, task_type, value, result)

    while True:
        try:
            sock_local.send(message)
            data = sock_local.recv(buffer_size)
            receive_frame = unpack(msg_fmt, data)
            TaskId = receive_frame[1]
            Result = receive_frame[4]
            sock_local.close()
            break
        except socket.error:
            print("Socket connection error, reconnecting...")
            time.sleep(3)
            sock_local = socketConnect()
        except:
            print("Other Error")
            time.sleep(3)
        time.sleep(1)

    return TaskId, Result


if __name__ == "__main__":
    TaskId, Result = agv_communication_lookup(task_id=30, value=1)
    print("Task Id: " + str(TaskId))
    print("Result: " + str(Result))
