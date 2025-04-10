#!/usr/bin/python3
# -*- coding: utf-8 -*-
import socket
import time
class AoyiHand():
    def __init__(self):
        # 右手
        ip = '169.254.128.19'
        port_no = 8080
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.connect((ip, port_no))
        # 初始化遨意灵巧手
        self.get_power_ready()
    # 傲意灵巧手modbus初始化
    def get_power_ready(self):
    
        point6_00 = '{"command":"set_modbus_mode","port":1,"baudrate":115200,"timeout ":2}\r\n'
        _ = self.send_cmd(cmd_6axis=point6_00)
        time.sleep(2)
        print("配置通讯端口通ModbusRTU")
        
    def send_cmd(self, cmd_6axis=''):

        self.client.send(cmd_6axis.encode('utf-8'))
    
        return True

    
    def open_hand(self):
        # 傲意灵巧手打开
        point6_00 = '{"command":"write_registers","port":1,"address":1135,"num":6,"data":[0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 0],"device":2}\r\n'
        _ = self.send_cmd(cmd_6axis=point6_00)
        time.sleep(1)
        print("傲意灵巧手打开")
    
    def catch_dumb(self):
        
        # 傲意灵巧手握住瓶子
        point6_00 = '{"command":"write_registers","port":1,"address":1135,"num":6,"data":[255,255,255,255,255,255,255,255,255,255,0,0],"device":2}\r\n'
        _ = self.send_cmd(cmd_6axis=point6_00)
        time.sleep(1)
        print("傲意灵巧手抓东西")

if __name__ == '__main__':
    # 测试
    aoyi_hand=AoyiHand()
    aoyi_hand.catch_dumb()
    aoyi_hand.open_hand()
