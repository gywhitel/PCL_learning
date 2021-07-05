import socket
import socketserver

# Server end

def socket_server():
    IP_port = ('127.0.0.1', 8001)
    # generate a handler 生成一个handler
    skt = socket.socket()
    skt.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

    skt.bind(IP_port)
    # maximum connection 最大连接数
    skt.listen(2)

    # 阻塞
    print("Waiting for connection...")
    #等待链接,阻塞，直到渠道链接 conn打开一个新的对象 专门给当前链接的客户端 addr是ip地址
    conn, address = skt.accept()
    print('Connection established. Client online.')
    conn.sendall(bytes("Connection established.", 'utf8'))

    # Flag = True
    point = []
    while True:
        # print("Waiting...")
        # accept data from client end
        client_data = conn.recv(1024)
        # print(str(client_data, 'utf8'))
        data = str(client_data, 'utf8')
        # data = data.replace('\n', '')
        data = data.split('\n')
        # QUESTION: 有一个接收发送时钟不同步的问题
        # point.append([float(data[0]), float(data[1])])
        data1 = data[0].split(' ')
        point.append( [float(data1[0]), float(data1[1])] )
        # data2 = data[1].split(' ')
        # point.append( [float(data2[0]), float(data2[1])] )

        # 向对方发送数据
        # conn.sendall(bytes("Copy that. Over.", encoding='utf-8'))
        # if client_data == 'exit':
        #     # Flag = False
        #     print('Abort communication.')
        #     conn.close()
        #     break
    print(point)


if __name__ == '__main__':
    socket_server()
'''
    IP_port = ('127.0.0.1', 8001)
    # generate a handler 生成一个handler
    skt = socket.socket()
    skt.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)

    skt.bind(IP_port)
    # maximum connection 最大连接数
    skt.listen(2)

    # 阻塞
    print("Waiting for connection...")
    #等待链接,阻塞，直到渠道链接 conn打开一个新的对象 专门给当前链接的客户端 addr是ip地址
    conn, address = skt.accept()
    print('Connection established. Client online.')
    conn.sendall(bytes("Connection established.", 'utf8'))

    # Flag = True
    while True:
        # print("Waiting...")
        # accept data from client end
        client_data = conn.recv(1024)
        print(str(client_data, 'utf8'))
        # 向对方发送数据
        conn.sendall(bytes("Copy that. Over.", encoding='utf-8'))
        if client_data == 'exit':
            # Flag = False
            print('Abort communication.')
            conn.close()
            break
    # 关闭连接
'''

'''

class MyServer(socketserver.BaseRequestHandler):
    
    def handle(self):
        print('Connection established')
        conn = self.request
        conn.sendall(bytes('This is a message from the server.', 'utf8'))
        Flag = True
        while Flag:
            print("waiting...")
            data = conn.recv(1024)
            if data == 'exit':
                Flag = False
            else:
                print(str(data, 'utf8'))
                # NOTE: 发送要用bytes格式
                conn.sendall(bytes('Copy that. Over.', 'utf8'))
if __name__ == '__main__':

    IP_port = ('127.0.0.1', 8000)
    server = socketserver.ThreadingTCPServer(IP_port, MyServer)
    server.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    print("Server online.")
    server.serve_forever()
'''