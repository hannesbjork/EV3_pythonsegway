#!/usr/bin/env python3

import socket
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import numpy as np


class LiveGraph4:

    def __init__(self, size):
        
        plt.style.use('ggplot')
        plt.ion()

        self.s = 100
        
        self.x_vec = np.linspace(0,1,size+1)[0:-1]
        self.y_vec1 = np.zeros(size)
        self.y_vec2 = np.zeros(size)
        self.y_vec3 = np.zeros(size)
        self.y_vec4 = np.zeros(size)
        
        self.figs, self.axs = plt.subplots(4)

        self.line1, = self.axs[0].plot(self.x_vec, self.y_vec1,'-o',alpha=0.8)
        self.line2, = self.axs[1].plot(self.x_vec, self.y_vec2,'-o',alpha=0.8)
        self.line3, = self.axs[2].plot(self.x_vec, self.y_vec3,'-o',alpha=0.8)
        self.line4, = self.axs[3].plot(self.x_vec, self.y_vec4,'-o',alpha=0.8)

        plt.show()

    def live_plot(self, y1, y2, y3, y4):
        

        self.line1.set_ydata(self.y_vec1)
        self.line2.set_ydata(self.y_vec2)
        self.line3.set_ydata(self.y_vec3)
        self.line4.set_ydata(self.y_vec4)

        #if np.min(self.y_vec1)<=self.line1.axes.get_ylim()[0] or np.max(self.y_vec1)>=self.line1.axes.get_ylim()[1]:
        #    self.axs[0].set_ylim([np.min(self.y_vec1)-np.std(self.y_vec1),np.max(self.y_vec1)+np.std(self.y_vec1)])

        self.set_graphlimit(self.y_vec1, self.line1, self.axs[0])
        self.set_graphlimit(self.y_vec2, self.line2, self.axs[1])
        self.set_graphlimit(self.y_vec3, self.line3, self.axs[2])
        self.set_graphlimit(self.y_vec4, self.line4, self.axs[3])

        self.y_vec1 = np.append(self.y_vec1[1:], y1)
        self.y_vec2 = np.append(self.y_vec2[1:], y2)
        self.y_vec3 = np.append(self.y_vec3[1:], y3)
        self.y_vec4 = np.append(self.y_vec4[1:], y4)

        plt.pause(0.05)

    def set_graphlimit(self, vector, line, axs):

        vstd = np.std(vector)
        vmax = np.max(vector)
        vmin = np.min(vector)

        if ( vmin <= line.axes.get_ylim()[0] ) or ( vmax >= line.axes.get_ylim()[1] ):
            axs.set_ylim( [vmin - vstd, vmax + vstd] )


class SimpleServer:
    def __init__(self, sock=None):
        if sock is None:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        else:
            self.sock = sock

    def openAndAccept(self, host, port):
        self.sock.bind((host, port))
        self.sock.listen(1)
        self.conn, addr = self.sock.accept()
        print('Connected by', addr)

    def recieve(self):
        with self.conn as conn:
            conn.recv(1024)

HOST = '10.42.0.1' 
PORT = 65432       

graph = LiveGraph4(100)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.bind((HOST, PORT))
    s.listen(1)
    conn, addr = s.accept()
    with conn:
        print('Connected by', addr)
        while True:
            data = conn.recv(1024)
            
            if data:    
                msg = data.decode('utf-8')
                
                y1 = float(msg.split(" ")[0])
                y2 = float(msg.split(" ")[1])
                y3 = float(msg.split(" ")[2])
                y4 = float(msg.split(" ")[3])
        
                graph.live_plot(y1, y2, y3, y4)




