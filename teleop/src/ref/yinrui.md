    # 相对位姿预测
    ssh连接docker环境
    连接跳板机 ==》docker，密码均为123456
    配置.ssh/config文件，添加以下主机：
    Host public
    HostName 2.tcp.vip.cpolar.cn
    User hanglok
    Port 13983
    IdentitiesOnly yes

    # Host pose
    HostName 127.0.0.1
    User root
    Port 20222
    ProxyCommand C:\Windows\System32\OpenSSH\ssh.exe -W %h:%p public

    # 位姿预测代码目录
    demo:
    /home/workspace/RelPoseRepo/demo.ipynb

    Model Zoo:
    /home/workspace/RelPoseRepo/models.py

