MotorCmd.msg => Kp 같은건 형식에 어긋남

kp로 수정

tauEst => tau_est

```
loading shared libraries: liblcm.so.1: cannot open shared object file: No such file or directory
```

=> sudo ldconfig -v

# 이더넷 포트 설정

https://askubuntu.com/questions/1049302/wired-ethernet-not-working-ubuntu-18-04

sudo lshw -C network
wlp2s0

cat /etc/network/interfaces


# Before
wlp2s0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.0.3  netmask 255.255.255.0  broadcast 192.168.0.255
        inet6 fe80::2472:cc68:8b05:e345  prefixlen 64  scopeid 0x20<link>
        ether 9c:b6:d0:8b:f5:43  txqueuelen 1000  (Ethernet)
        RX packets 97500  bytes 105630048 (105.6 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 58785  bytes 13845106 (13.8 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0

# After
wlp2s0: flags=4163<UP,BROADCAST,RUNNING,MULTICAST>  mtu 1500
        inet 192.168.123.162  netmask 255.255.255.0  broadcast 192.168.123.255
        ether 9c:b6:d0:8b:f5:43  txqueuelen 1000  (Ethernet)
        RX packets 110509  bytes 118249226 (118.2 MB)
        RX errors 0  dropped 0  overruns 0  frame 0
        TX packets 66910  bytes 15486905 (15.4 MB)
        TX errors 0  dropped 0 overruns 0  carrier 0  collisions 0


$ ros2 run unitree_legged_real lcm_server LOWLEVEL
```
[Error] Connect failed: Network is unreachable
target IP:192.168.123.10
UDP Initialized. socketfd: 3    Port: 8080 
LCM Initialized. Subscribe channel: LCM_Low_Cmd, Publish channel: LCM_Low_State
[Loop Start] named: UDP_Send, period: 2(ms), run at cpu: 3 
[Loop Start] named: UDP_Recv, period: 2(ms), run at cpu: 3 
[Loop Start] named: LCM_Recv, period: 2(ms), cpu unspecified
[Loop Start] named: control_loop, period: 2(ms), cpu unspecified
```


=> 이건 A1이라서 그런 듯

# Joy

```
sudo apt install ros-eloquent-joy

ros2 run joy joy_node --ros-args -p autorepeat_rate:=100.0
```

