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
        <param field="Mode3" label="Reading Interval min." width="40px" required="true" default="1" />
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
                         payload = RS485.read_register(self.register,self.nod)
                 elif self.size == 2:
                         payload = RS485.read_long(self.register)
                         if self.nod == 1:
                            payload = payload / 10
                         elif self.nod == 2:
                            payload = payload / 100   
                 data = payload
                 Domoticz.Log("Device:"+self.name+" data="+str(data)+" from register: "+str(self.register) )                        

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
        self.RS485.debug = True
        self.RS485.mode = minimalmodbus.MODE_RTU
        
        devicecreated = []
        Domoticz.Log("DDS238-7 Modbus plugin start")
# ID,name,type,number_of_decimals,register,size,signed:bool=False,options=None):

        self.devs = [
                 Dev(1,"LifeEnergy","kWh",2,0,2,False,None),
                 Dev(2,"TotalReactivePower","kWh",2,2,2,False,None),
                 Dev(3,"Foo","Usage",2,4,2,False,None),                 
                 Dev(4,"Bar","Usage",2,6,2,False,None),
                 Dev(5,"RevEnergy","kWh",2,8,2,False,None),
                 Dev(6,"FwdEnergy","kWh",2,10,2,False,None),
                 Dev(7,"Voltage Frequency","Custom",2,17,1,False,{ "Custom" : "1;Hz"}),                 
                 Dev(8,"Voltage_L1","Voltage",1,128,1,False,None),
                 Dev(9,"Voltage_L2","Voltage",1,129,1,False,None),
                 Dev(10,"Voltage_L3","Voltage",1,130,1,False,None),
                 Dev(11,"Current_L1","Current (Single)",2,131,1,False,None),
                 Dev(12,"Current_L2","Current (Single)",2,132,1,False,None),
                 Dev(13,"Current_L3","Current (Single)",2,133,1,False,None),
                 Dev(14,"TotalCurrentPower","Usage",2,135,2,False,None),
                 Dev(15,"CurrentPowerA","Usage",0,136,1,False,None),
                 Dev(16,"CurrentPowerB","Usage",0,137,1,False,None),
                 Dev(17,"CurrentPowerC","Usage",0,138,1,False,None)
            ]

                
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
