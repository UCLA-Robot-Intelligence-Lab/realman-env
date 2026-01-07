#!/usr/bin/python3
# -*- coding: utf-8 -*-
import socket
import time
class AoyiHand():
    def __init__(self):
        # 右手
        ip = '169.254.128.19' # 18- left 19 - right
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
   
    def thumb4(self):    
        # 傲意灵巧手握住瓶子
        point6_00 = '{"command":"write_registers","port":1,"address":1135,"num":6,"data":[0,0,0,125,0,0,0,0,0,0,0,0],"device":2}\r\n'
        # 0,1 for thumb bend 0 is inward and 1 is outward
        # 2,3 for index bend 2 is inward and 3 is outward
        # 4,5 for middle bend 4 is inward and 5 is outward
        # 6,7 for ring bend 6 is inward and 7 is outward
        # 8,9 for little bend 8 is inward and 9 is outward
        # 10 for thumb rotation. Absolute Position. Range: 0~255
        # 11 Needs Test
        _ = self.send_cmd(cmd_6axis=point6_00)
        time.sleep(1)

if __name__ == '__main__':
    # 测试
    aoyi_hand=AoyiHand()
    # thumb bend is 0 , 1 
    # one closes it one opens it
    # thumb rotation is 10
    # same thing as above
    # i changed the values just for demo but the other indices are for the other fingers clsoe and open
    aoyi_hand.thumb4()
    time.sleep(1)
