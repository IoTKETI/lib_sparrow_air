# lib_sparrow_air
드론에 탑재된 환경센서로 대기환경을 센싱, 제어하는 라이브러리 프로세스

## Install dependencies
### MQTT-broker
```
$ wget http://repo.mosquitto.org/debian/mosquitto-repo.gpg.key
$ sudo apt-key add mosquitto-repo.gpg.key
$ cd /etc/apt/sources.list.d/
$ sudo wget http://repo.mosquitto.org/debian/mosquitto-buster.list 
$ sudo apt-get update
$ sudo apt-get install -y mosquitto
```
### Python Library
#### mqtt
```
$ pip3 install paho-mqtt
