#!/usr/bin/python3
import json, sys, serial, threading, time, os, signal
import paho.mqtt.client as mqtt

i_pid = os.getpid()
argv = sys.argv

air_event = 0x00

CONTROL_E = 0x01
DATA_E = 0x02

control_topic = ''
data_topic = ''

airQ = {}
missionPort = None
con = ''
count = 0
broker_ip = 'localhost'
port = 1883

def airQ_init():
    airQ['PM25'] = 0.0 # (ug/m3)
    airQ['PM10'] = 0.0 # (ug/m3)
    airQ['CO'] = 0.0 # (ppb)
    airQ['NO2'] = 0.0 # (ppb)
    airQ['O3_org'] = 0.0 # (org/ppb)
    airQ['O3_comp'] = 0.0 # (comp/ppb)
    airQ['SO2_org'] = 0.0 # (org/ppb)
    airQ['SO2_comp'] = 0.0 # (comp/ppb)
    airQ['T'] = 0.0 # (C)
    airQ['H'] = 0.0 # (%)
    airQ['NO2_OP1'] = 0 # (mV)
    airQ['NO2_OP2'] = 0 # (mV)
    airQ['O3_OP1'] = 0 # (mV)
    airQ['O3_OP2'] = 0 # (mV)
    airQ['CO_OP1'] = 0 # (mV)
    airQ['CO_OP2'] = 0 # (mV)
    airQ['SO2_OP1'] = 0 # (mV)
    airQ['SO2_OP2'] = 0 # (mV)


def on_connect(client,userdata,flags, rc):
    print('[msw_mqtt_connect] connect to ', broker_ip)
    lib_mqtt_client.subscribe(control_topic, 0)
    print ('[lib]control_topic\n', control_topic)


def on_disconnect(client, userdata, flags, rc=0):
	print(str(rc))


def on_subscribe(client, userdata, mid, granted_qos):
    print("subscribed: " + str(mid) + " " + str(granted_qos))


def on_message(client, userdata, msg):
    global missionPort
    global air_event
    global data_topic
    global control_topic
    global con

    if msg.topic == control_topic:
        con = str(msg.payload.decode("utf-8"))
        print(con)
        if (con == 'G'):
            air_event |= CONTROL_E
        elif (con == '1'):
            air_event |= DATA_E
    

def on_receive_from_msw(str_message):
    global missionPort

    setcmd = b'G'
    print('setcmd: ', str_message)
    missionPort.write(setcmd)


def msw_mqtt_connect(broker_ip, port):
    global lib_topic
    global lib_mqtt_client

    lib_mqtt_client = mqtt.Client()
    lib_mqtt_client.on_connect = on_connect
    lib_mqtt_client.on_disconnect = on_disconnect
    lib_mqtt_client.on_subscribe = on_subscribe
    lib_mqtt_client.on_message = on_message
    lib_mqtt_client.connect(broker_ip, port)
    lib_mqtt_client.loop_start()
    return lib_mqtt_client


def missionPortOpening(missionPortNum, missionBaudrate):
    global airQ
    global lib
    global missionPort

    if (missionPort == None):
        try:
            missionPort = serial.Serial(missionPortNum, missionBaudrate, timeout = 2)
            print ('missionPort open. ' + missionPortNum + ' Data rate: ' + missionBaudrate)

        except TypeError as e:
            missionPortClose()
    else:
        if (missionPort.is_open == False):
            missionPortOpen()

            # airQ.rssi = -Math.random()*100;
            send_data_to_msw(data_topic, airQ)

def missionPortOpen():
    print('missionPort open!')
    missionPort.open()

def missionPortClose():
    global missionPort
    print('missionPort closed!')
    missionPort.close()


def missionPortError(err):
    print('[missionPort error]: ', err)
    os.kill(i_pid, signal.SIGKILL)

def airReqMessage():
    global missionPort

    if missionPort is not None:
        if missionPort.is_open:
            setcmd = b'I'
            missionPort.write(setcmd)
            flag = 0


def send_data_to_msw (data_topic, obj_data):
    global lib_mqtt_client
    
    lib_mqtt_client.publish(data_topic, obj_data)


def missionPortData():
    global missionPort
    global airQ
    global count
    global data_topic

    airReqMessage()
    missionStr = missionPort.readlines()
    print('origin: ', missionStr)
    try:
        if ((not missionStr) or (missionStr[0] == b'\x00\n')):
            airReqMessage()

            missionStr = missionPort.readlines()
            print('origin: ', missionStr)

            arrAIRQ = missionStr[3].decode("utf-8").replace(" ", "")
            arrQValue = arrAIRQ.split(',')
            airQ['PM25'] = float(arrQValue[0])  # (ug/m3)
            airQ['PM10'] = float(arrQValue[1])  # (ug/m3)
            airQ['CO'] = float(arrQValue[2])  # (ppb)
            airQ['NO2'] = float(arrQValue[3])  # (ppb)
            airQ['O3_org'] = float(arrQValue[4])  # (org/ppb)
            airQ['O3_comp'] = float(arrQValue[5])  # (comp/ppb)
            airQ['SO2_org'] = float(arrQValue[6])  # (org/ppb)
            airQ['SO2_comp'] = float(arrQValue[7])  # (comp/ppb)
            airQ['T'] = float(arrQValue[8])  # (C)
            airQ['H'] = float(arrQValue[9])  # (%)
            airQ['NO2_OP1'] = int(arrQValue[10])  # (mV)
            airQ['NO2_OP2'] = int(arrQValue[11])  # (mV)
            airQ['O3_OP1'] = int(arrQValue[12])  # (mV)
            airQ['O3_OP2'] = int(arrQValue[13])  # (mV)
            airQ['CO_OP1'] = int(arrQValue[14])  # (mV)
            airQ['CO_OP2'] = int(arrQValue[15])  # (mV)
            airQ['SO2_OP1'] = int(arrQValue[16])  # (mV)
            airQ['SO2_OP2'] = int(arrQValue[17])  # (mV)

            airQ = json.dumps(airQ)
            send_data_to_msw(data_topic, airQ)
            print('airQ: ', missionStr)
            airQ = json.loads(airQ)

        else:
            arrAIRQ = missionStr[3].decode("utf-8").replace(" ","")
            arrQValue = arrAIRQ.split(',')
            airQ['PM25'] = float(arrQValue[0])  # (ug/m3)
            airQ['PM10'] = float(arrQValue[1])  # (ug/m3)
            airQ['CO'] = float(arrQValue[2])  # (ppb)
            airQ['NO2'] = float(arrQValue[3])  # (ppb)
            airQ['O3_org'] = float(arrQValue[4])  # (org/ppb)
            airQ['O3_comp'] = float(arrQValue[5])  # (comp/ppb)
            airQ['SO2_org'] = float(arrQValue[6])  # (org/ppb)
            airQ['SO2_comp'] = float(arrQValue[7])  # (comp/ppb)
            airQ['T'] = float(arrQValue[8])  # (C)
            airQ['H'] = float(arrQValue[9])  # (%)
            airQ['NO2_OP1'] = int(arrQValue[10])  # (mV)
            airQ['NO2_OP2'] = int(arrQValue[11])  # (mV)
            airQ['O3_OP1'] = int(arrQValue[12])  # (mV)
            airQ['O3_OP2'] = int(arrQValue[13])  # (mV)
            airQ['CO_OP1'] = int(arrQValue[14])  # (mV)
            airQ['CO_OP2'] = int(arrQValue[15])  # (mV)
            airQ['SO2_OP1'] = int(arrQValue[16])  # (mV)
            airQ['SO2_OP2'] = int(arrQValue[17])  # (mV)

            airQ = json.dumps(airQ)
            send_data_to_msw(data_topic, airQ)
            print('airQ: ', missionStr)
            airQ = json.loads(airQ)


    except (ValueError, IndexError):
        airQ_init()
        airReqMessage()
        pass

    except serial.SerialException as e:
        missionPortError(e)


def main():
    global lib
    global lib_mqtt_client
    global req_topic
    global broker_ip
    global port
    global air_event
    global con
    global control_topic
    global data_topic

    my_lib_name = 'lib_sparrow_air'

    try:
        lib = dict()
        with open(my_lib_name + '.json', 'r') as f:
            lib = json.load(f)
            lib = json.loads(lib)

    except:
        lib = dict()
        lib["name"] = my_lib_name
        lib["target"] = 'armv6'
        lib["description"] = "[name] [portnum] [baudrate]"
        lib["scripts"] = './' + my_lib_name + ' /dev/ttyUSB3 115200'
        lib["data"] = ['AIR']
        lib["control"] = ['Control_AIR']
        lib = json.dumps(lib, indent=4)
        lib = json.loads(lib)

        with open('./' + my_lib_name + '.json', 'w', encoding='utf-8') as json_file:
            json.dump(lib, json_file, indent=4)


    lib['serialPortNum'] = argv[1]
    lib['serialBaudrate'] = argv[2]

    control_topic = '/MUV/control/' + lib["name"] + '/' + lib["control"][0]
    data_topic = '/MUV/data/' + lib["name"] + '/' + lib["data"][0]

    msw_mqtt_connect(broker_ip, port)

    missionPortNum = lib["serialPortNum"]
    missionBaudrate = lib["serialBaudrate"]
    missionPortOpening(missionPortNum, missionBaudrate)

    airQ_init()
    missionPortData()

    while True:
        if air_event & CONTROL_E:
            air_event &= (~CONTROL_E)
            on_receive_from_msw(con)
        elif air_event & DATA_E:
            air_event &= (~DATA_E)
            missionPortData()


if __name__ == '__main__':
    main()
