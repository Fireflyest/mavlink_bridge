# pc防火墙设置
netsh advfirewall firewall add rule name="MAVLink UDP" dir=in action=allow protocol=UDP localport=14555
netsh advfirewall firewall add rule name="MAVLink Ping" dir=in action=allow protocol=icmpv4
