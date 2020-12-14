# opil_san_sensors multiple

# Overview

This package pretends to be a bridge between ROS and Sensor Agent Node using multiple instances.



trigger_0 -> MOD_BUTTON_G1  Botón verde de la estación 1
trigger_1 -> MOD_BUTTON_R1  Botón rojo de la estación 1
trigger_2 -> MOD_BUTTON_G2  Botón verde de la estación 2
trigger_3 -> MOD_BUTTON_R2  Botón rojo de la estación 2
trigger_4 -> MOD_FDC_1      Final de carrera 1 (Puerta arriba)
trigger_5 -> MOD_FDC_2      Final de carrera 2 (Puerta abajo)
trigger_6 -> MOD_PHOTO_1    Fotocélula puesto 1 alamacén
trigger_7 -> MOD_PHOTO_2    Fotocélula puesto 1 alamacén
trigger_8 -> MOD_PHOTO_3    Fotocélula puesto 1 alamacén

# Instructions 

1. Launch the opil server. Check the ip address.

```
$ sudo docker-compose up
```

2. Configure san socket. In the follow directory set the ip address of the opil server

```
$ opil_san_sensors/docker/config.json
```

Change ```ip-address``` by your opil server address. Note that quotes are required.

```
{
    "contextBroker": {
        "host": "ip-address",
        "port": "1026"
    ...
...   
```

Then, start docker container in ```opil_san_sensors/docker/docker-compose.yml```

```
$ sudo docker-compose up
```





3. Launch ROS client for SAN module comunication

```
$ roslaunch opil_san_sensors client.launch 
```

4. Send a signal from ROS to OPIL server using SAN module. This package creates a ROS
service called ```opil_san_sensors/trigger ```. Use it to send a flag to OPIL Server.

```
$ rosservice call /opil_san_sensors/trigger "{}"
```
