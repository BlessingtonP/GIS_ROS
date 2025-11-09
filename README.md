# GIS_ROS

In Rounting_Widget.h:
publicL
osgEarth::MapNode* m_mapNode = nullptr;
 
Get the mapNode from constructor of Routing_Widget.
 
Routing_Widget where it gets the mapNode:
In mainwindow.cpp
 
Line 1809:
m_routingWidget = new Routing_Widget(m_mapView, ThreeDMapScene::getInstance()->getSceneManager()->getMapNode());


#IMPORTANT - STOP FIREWALL For the Receiving part, otherwise, packets will be received in the Machine but not inside the application.

[mglocadmin@localhost ~]$ sudo systemctl disable firewalld
[sudo] password for mglocadmin: 
Removed "/etc/systemd/system/multi-user.target.wants/firewalld.service".
Removed "/etc/systemd/system/dbus-org.fedoraproject.FirewallD1.service".
[mglocadmin@localhost ~]$ sudo systemctl is-enabled firewalld
disabled
[mglocadmin@localhost ~]$ sudo systemctl status firewalld
● firewalld.service - firewalld - dynamic firewall daemon
     Loaded: loaded (/usr/lib/systemd/system/firewalld.service; disabled; vendor preset: enabled)
     Active: active (running) since Mon 2025-11-10 00:28:49 IST; 2h 23min ago
       Docs: man:firewalld(1)
   Main PID: 1302 (firewalld)
      Tasks: 2 (limit: 98174)
     Memory: 42.5M
        CPU: 407ms
     CGroup: /system.slice/firewalld.service
             └─1302 /usr/bin/python3 -s /usr/sbin/firewalld --nofork --nopid

Nov 10 00:28:45 localhost systemd[1]: Starting firewalld - dynamic firewall daemon...
Nov 10 00:28:49 localhost systemd[1]: Started firewalld - dynamic firewall daemon.
[mglocadmin@localhost ~]$ sudo systemctl stop firewalld
[mglocadmin@localhost ~]$ sudo systemctl status firewalld
○ firewalld.service - firewalld - dynamic firewall daemon
     Loaded: loaded (/usr/lib/systemd/system/firewalld.service; disabled; vendor preset: enabled)
     Active: inactive (dead) since Mon 2025-11-10 02:52:34 IST; 2s ago
   Duration: 2h 23min 44.346s
       Docs: man:firewalld(1)
    Process: 1302 ExecStart=/usr/sbin/firewalld --nofork --nopid $FIREWALLD_ARGS (code=exited, status=0/SUCCESS)
   Main PID: 1302 (code=exited, status=0/SUCCESS)
        CPU: 458ms

Nov 10 00:28:45 localhost systemd[1]: Starting firewalld - dynamic firewall daemon...
Nov 10 00:28:49 localhost systemd[1]: Started firewalld - dynamic firewall daemon.
Nov 10 02:52:34 localhost.localdomain systemd[1]: Stopping firewalld - dynamic firewall daemon...
Nov 10 02:52:34 localhost.localdomain systemd[1]: firewalld.service: Deactivated successfully.
Nov 10 02:52:34 localhost.localdomain systemd[1]: Stopped firewalld - dynamic firewall daemon.
[mglocadmin@localhost ~]$ sudo systemctl disable firewalld
[mglocadmin@localhost ~]$ sudo systemctl status firewalld
○ firewalld.service - firewalld - dynamic firewall daemon
     Loaded: loaded (/usr/lib/systemd/system/firewalld.service; disabled; vendor preset: enabled)
     Active: inactive (dead) since Mon 2025-11-10 02:52:34 IST; 11s ago
   Duration: 2h 23min 44.346s
       Docs: man:firewalld(1)
   Main PID: 1302 (code=exited, status=0/SUCCESS)
        CPU: 458ms

Nov 10 00:28:45 localhost systemd[1]: Starting firewalld - dynamic firewall daemon...
Nov 10 00:28:49 localhost systemd[1]: Started firewalld - dynamic firewall daemon.
Nov 10 02:52:34 localhost.localdomain systemd[1]: Stopping firewalld - dynamic firewall daemon...
Nov 10 02:52:34 localhost.localdomain systemd[1]: firewalld.service: Deactivated successfully.
Nov 10 02:52:34 localhost.localdomain systemd[1]: Stopped firewalld - dynamic firewall daemon.

