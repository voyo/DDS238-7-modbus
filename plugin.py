#!/usr/bin/env python
"""
DDS238-7 Three Phase Smart Meter. Domoticz plugin.
Author: voyo@no-ip.pl
Requirements: 
    1.python module minimalmodbus -> http://minimalmodbus.readthedocs.io/en/master/
        (pi@raspberrypi:~$ sudo pip3 install minimalmodbus)
    2.Communication module Modbus USB to RS485 converter module
"""
"""
<plugin key="DDS238-7" name="DDS238-7-Modbus" version="0.1" author="voyo@no-ip.pl">
    <params>
        <param field="SerialPort" label="Modbus Port" width="200px" required="true" default="/dev/ttyUSB0" />
        <param field="Mode1" label="Baud rate" width="40px" required="true" default="9600"  />
        <param field="Mode2" label="Device ID" width="40px" required="true" default="1" />
        <param field="Mode3" label="Reading Interval * 10s." width="40px" required="true" default="1" />
        <param field="Mode6" label="Debug" width="75px">
            <options>
                <option label="True" value="Debug"/>
                <option label="False" value="Normal"  default="true" />
            </options>
        </param>
    </params>
</plugin>

"""

import minimalmodbus
import serial
import Domoticz
from time import sleep


class Dev:
    def __init__(self,ID,name,type,nod,register,size,signed:bool=False,options=None):
        self.ID = ID
        self.name = name
        self.type = type
        self.nod = nod
        self.value = 0
        self.signed = signed        
        self.register = register
        self.size = size
        self.options = options if options is not None else None
        msg = "Registering device: "+self.name+" "+str(self.ID)+" "+self.type;
        Domoticz.Log(msg)
        if self.ID not in Devices:
             Domoticz.Device(Name=self.name, Unit=self.ID, TypeName=self.type,Used=1,Options=self.options).Create()


    def UpdateValue(self,RS485):
                 if self.size == 1:
                     while True:
                       try:
                           payload = RS485.read_register(self.register,self.nod)
                       except Exception as e:
#                           Domoticz.Log("Connection failure: "+str(e))
                           Domoticz.Log("Modbus connection failure")
                           Domoticz.Log("retry updating register in 2 s")
                           sleep(2.0)
                           continue
                       break
                 elif self.size == 2:
                         while True:
                           try:
                             payload = RS485.read_long(self.register)                           
                           except Exception as e:
#                           Domoticz.Log("Connection failure: "+str(e))
                               Domoticz.Log("Modbus connection failure")
                               Domoticz.Log("retry updating register in 2 s")
                               sleep(2.0)
                               continue
                           break
                         if self.nod == 1:
                            payload = payload / 10
                         elif self.nod == 2:
                            payload = payload / 100   
                 data = payload
                 Domoticz.Log("Device:"+self.name+" data="+str(data)+" from register: "+str(hex(self.register)) )

#                 Devices[self.ID].Update(sValue=str(data))
                 Devices[self.ID].Update(0,str(data)+';0',True)
#             self.updateDevice(deviceUnitId, 0, str(valDouble)+';0', True) # force update, even if the voltage has no changed. 

                 if Parameters["Mode6"] == 'Debug':
                     Domoticz.Debug("DEBUG in UpdateValue "+self.name+' {0:.3f} V'.format(data))
                    


class BasePlugin:
    def __init__(self):
        self.runInterval = 1
        self.RS485 = ""
        return

    def onStart(self):
        self.RS485 = minimalmodbus.Instrument(Parameters["SerialPort"], int(Parameters["Mode2"]))
        self.RS485.serial.baudrate = Parameters["Mode1"]
        self.RS485.serial.bytesize = 8
        self.RS485.serial.parity = minimalmodbus.serial.PARITY_NONE
        self.RS485.serial.stopbits = 1
        self.RS485.serial.timeout = 1
        self.RS485.debug = False
        self.RS485.mode = minimalmodbus.MODE_RTU
        
        devicecreated = []
        Domoticz.Log("DDS238-7 Modbus plugin start")
# ID,name,type,number_of_decimals,register,size,signed:bool=False,options=None):

        self.devs = [
                 Dev(1,"LifeEnergy","kWh",2,0x00,2,False,None),
                 Dev(2,"TotalReactivePower","kWh",2,0x02,2,False,None),
                 Dev(3,"Foo","Usage",2,0x04,2,False,None),                 
                 Dev(4,"Bar","Usage",2,0x06,2,False,None),
                 Dev(5,"RevEnergy","kWh",2,0x08,2,False,None),
                 Dev(6,"FwdEnergy","kWh",2,0x10,2,False,None),
                 Dev(7,"Voltage Frequency","Custom",2,0x11,1,False,{ "Custom" : "1;Hz"}),                 
                 Dev(8,"Voltage_L1","Voltage",1,0x80,1,False,None),
                 Dev(9,"Voltage_L2","Voltage",1,0x81,1,False,None),
                 Dev(10,"Voltage_L3","Voltage",1,0x82,1,False,None),
                 Dev(11,"Current_L1","Current (Single)",2,0x83,1,False,None),
                 Dev(12,"Current_L2","Current (Single)",2,0x84,1,False,None),
                 Dev(13,"Current_L3","Current (Single)",2,0x85,1,False,None),
                 Dev(14,"TotalActivePower","Usage",2,0x86,2,False,None),
                 Dev(15,"TotalPower","Usage",2,0x87,2,False,None),
                 Dev(16,"ActivePowerA","Usage",0,0x88,1,False,None),
                 Dev(17,"ActivePowerB","Usage",0,0x89,1,False,None),
                 Dev(18,"ActivePowerC","Usage",0,0x8A,1,False,None),
                 Dev(19,"TotalReactivePower","Usage",2,0x8C,1,False,None),
                 Dev(20,"ReactivePowerA","Usage",0,0x8D,1,False,None),
                 Dev(21,"ReactivePowerB","Usage",0,0x8E,1,False,None),
                 Dev(22,"ReactivePowerC","Usage",0,0x8F,1,False,None), 
                 Dev(23,"TotalApparentPower","Usage",2,0x90,1,False,None),
                 Dev(24,"ApparentPowerA","Usage",0,0x92,1,False,None),
                 Dev(25,"ApparentPowerB","Usage",0,0x93,1,False,None),
                 Dev(26,"ApparentPowerC","Usage",0,0x94,1,False,None),
                 Dev(27,"PowerFactor","Usage",3,0x95,1,False,None),
                 Dev(28,"PowerFactorA","Usage",3,0x96,1,False,None),
                 Dev(29,"PowerFactorB","Usage",3,0x97,1,False,None),
                 Dev(30,"PowerFactorC","Usage",3,0x98,1,False,None)
            ]
#{ "Custom" : "1;factor}),
                
    def onStop(self):
        Domoticz.Log("DDS238-7 Modbus plugin stop")

    def onHeartbeat(self):
        self.runInterval -=1;
        if self.runInterval <= 0:
            for i in self.devs:
                try:
                         # Get data from modbus
                        Domoticz.Debug("Getting data from modbus for device:"+i.name+" ID:"+str(i.ID))
# tutaj
                        self.devs[i.ID-1].UpdateValue(self.RS485)
                except Exception as e:
                        Domoticz.Log("Connection failure: "+str(e));
                else:
#                        Devices[i.ID].Update(0,str(i.value))
#                        Devices[i.ID].Update(nValue=i.value,sValue=str(i.value))
                        if Parameters["Mode6"] == 'Debug':
                            Domoticz.Debug("in HeartBeat "+i.name+": "+format(i.value))
            Domoticz.Log("DDS238-7 set INTERVAL to: " + str(Parameters["Mode3"]) )
            self.runInterval = int(Parameters["Mode3"]) 


global _plugin
_plugin = BasePlugin()


def onStart():
    global _plugin
    _plugin.onStart()


def onStop():
    global _plugin
    _plugin.onStop()


def onHeartbeat():
    global _plugin
    Domoticz.Debug("onHeartbeat called")
    _plugin.onHeartbeat()

# Generic helper functions
def DumpConfigToLog():
    for x in Parameters:
        if Parameters[x] != "":
            Domoticz.Debug("'" + x + "':'" + str(Parameters[x]) + "'")
    Domoticz.Debug("Device count: " + str(len(Devices)))
    for x in Devices:
        Domoticz.Debug("Device:           " + str(x) + " - " + str(Devices[x]))
        Domoticz.Debug("Device ID:       '" + str(Devices[x].ID) + "'")
        Domoticz.Debug("Device Name:     '" + Devices[x].Name + "'")
        Domoticz.Debug("Device nValue:    " + str(Devices[x].nValue))
        Domoticz.Debug("Device sValue:   '" + Devices[x].sValue + "'")
        Domoticz.Debug("Device LastLevel: " + str(Devices[x].LastLevel))
    return
