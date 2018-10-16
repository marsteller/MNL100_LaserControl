# -*- coding: utf-8 -*-
"""
Created on Mon Oct 15 16:40:11 2018

@author: Alexander Marsteller
"""

import serial
from PyQt5.QtCore import (QCoreApplication, QObject, QRunnable, QThread,
                          QThreadPool, pyqtSignal, pyqtSlot)
import sys
import time
import numpy as np

python_version = float(sys.version_info.major)
serial_version = float(serial.__version__)

print(python_version)
print(serial_version)


class LaserCommunicationHandler(object):
    
    
    def __init__(self):
        
        #define variables
        self.request_start_delimiter = "#"
        self.response_start_delimiter = "<"
        self.destination_address = "!"
        self.source_address = "@"
        self.end_delimiter = "\r"
        
        
        self.command_dictionary =  {}
        self.command_dictionary["LaserOff"] = "X"
        self.command_dictionary["LaserOn"] = "g"
        self.command_dictionary["RepetitionOn"] = "h"
        self.command_dictionary["BurstOn"] = "j"
        self.command_dictionary["ExtTrigmode"] = "u"
        self.command_dictionary["LaserStop"] = "i"
        self.command_dictionary["SetBurstQuantity"] = "l"
        self.command_dictionary["SetRepetitionFrequency"] = "m"
        self.command_dictionary["SetHV"] = "n"
        self.command_dictionary["IncrementHV"] = "o1"
        self.command_dictionary["DecrementHV"] = "o0"
        self.command_dictionary["SetShutter"] = "z"
        self.command_dictionary["SetStepperPosition"] = "O3"
        self.command_dictionary["SetTransmission"] = "O4"
        self.command_dictionary["SetAttenuationEnergy"] = "O5"
        self.command_dictionary["InitAttenuator"] = "O60000"
        
        self.command_dictionary["GetShortStatus"] = "W"
        self.command_dictionary["GetStat7"] = "UT"
        self.command_dictionary["GetStat8"] = "UU"
        self.command_dictionary["GetVer3"] = "V3"
        self.command_dictionary["GetSernum"] = "US"
        self.command_dictionary["GetAttenuatorStatus"] = "UV"
        self.command_dictionary["GetEnergyValues"] = "P"
        
        
        
        
        self.command_parameter_dictionary = {}
        self.command_parameter_dictionary["LaserOff"] = None
        self.command_parameter_dictionary["LaserOn"] = None
        self.command_parameter_dictionary["RepetitionOn"] = None
        self.command_parameter_dictionary["BurstOn"] = None
        self.command_parameter_dictionary["ExtTrigmode"] = None
        self.command_parameter_dictionary["LaserStop"] = None
        self.command_parameter_dictionary["SetBurstQuantity"] = {"min":0, "max":9999, "length":4}
        self.command_parameter_dictionary["SetRepetitionFrequency"] = {"min":0, "max":99, "length":2}
        self.command_parameter_dictionary["SetHV"] = {"min":0, "max":100, "length":2}
        self.command_parameter_dictionary["IncrementHV"] = None
        self.command_parameter_dictionary["DecrementHV"] = None
        self.command_parameter_dictionary["SetShutter"] = {"min":0, "max":1, "length":1}
        self.command_parameter_dictionary["SetStepperPosition"] = {"min":0, "max":399, "length":4}
        self.command_parameter_dictionary["SetTransmission"] = {"min":0, "max":200, "length":2}
        self.command_parameter_dictionary["SetAttenuationEnergy"] = {"min":0, "max":9999, "length":4}
        self.command_parameter_dictionary["InitAttenuator"] = None
        
        
        self.reply_directory = {}
        self.reply_directory["W"] = "GetShortStatus"
        self.reply_directory["UT"] = "GetStat7"
        self.reply_directory["UU"] = "GetStat8"
        self.reply_directory["US"] = "GetSernum"
        self.reply_directory["UV"] = "GetAttenuatorStatus"
        self.reply_directory["P"] = "GetEnergyValues"
        self.reply_directory["V"] = "GetVer3"
        
        
        #define more stuff
        
        
        self.energy_values = []
        
        
        self.shutter_status = 0
        
        
    def _calculate_frame_check_squence(self, telegram):
        
        fcs = hex(sum(bytearray(telegram.encode("ASCII"))) % 256).upper()[2:]
        return fcs
        
    
    def compose_command(self, command_string, command_parameter=None):
        command = self.request_start_delimiter + self.destination_address + self.source_address
        command = command + self.command_dictionary[command_string]
        
        if command_parameter != None and self.command_parameter_dictionary[command_string] != None:
            if command_parameter <= self.command_parameter_dictionary[command_string]["max"] and command_parameter >= self.command_parameter_dictionary[command_string]["min"]:
                #parameter_string = hex(command_parameter).upper()
                
                parameter_string = "{0:0{1}x}".format(command_parameter,self.command_parameter_dictionary[command_string]["length"]).upper()
                command += parameter_string
            else:
                raise ValueError()
        
        command = command + self._calculate_frame_check_squence(command)
        command = command + self.end_delimiter
        
        return command
        
        
        
    def _interprete_response(self, reply):
        
        cleaned_reply = reply.rstrip(self.end_delimiter)
        reply_fcs = cleaned_reply[-2:]
        
        
        calculated_reply_fcs = self._calculate_frame_check_squence(cleaned_reply[:-2])
        
        if reply_fcs != calculated_reply_fcs:
            raise IOError()
        else:
            cleaned_reply = cleaned_reply.rstrip(reply_fcs)
        
        cleaned_reply = cleaned_reply.lstrip(self.response_start_delimiter).lstrip(self.source_address).lstrip(self.destination_address)
        
        response_type = None
        for key in self.reply_directory.keys():
            if cleaned_reply[:len(key)] == key:
                response_type = self.reply_directory[key]
                break
        
        if response_type != None:
            eval("self._interprete_{}(cleaned_reply)".format(response_type))
        
    def _interprete_GetShortStatus(self, reply):
        cleaned_reply = reply[1:]
        
        response_dict = {"00":"Standby", "01":"Laser is Working", "03":"EEPROM Error",
                         "04":"Energy Monitor Error occured", "05":"Temperature too high (>48°C)",
                         "06":"Static Error", "07":"Operation Error: Laser mus be switched off"}
        
        return response_dict[cleaned_reply]
    
    def _interprete_GetStat7(self, reply):
        cleaned_reply = cleaned_reply[2:]
        
        flag_byte_1 = int(cleaned_reply[:2],16)
        #flag_byte_2 = reply[2:4]   # unused
        flag_byte_3 = int(cleaned_reply[4:6],16)
        quantity = int(cleaned_reply[6:10],16)
        frequency = int(cleaned_reply[10:12],16)
        hv = int(cleaned_reply[12:14],16)
        energy = int(cleaned_reply[18:22],16)
        
        flag_bytes_1 = {0:"Shutter is Open", 2:"Laser is Ready for Operation", 3:"Laser Standby", 
                      4:"LaserMode: Off", 5:"LaserMode: Repetition", 6:"LaserMode: Burst", 7:"LaserMode: External triggering"}
        flag_bytes_3 = {0:"Service Mode activated", 5:"EEPROM error", 6:"Watchdog Reset occurred"}
        
        if flag_byte_1 == 0:
            self.shutter_status = 1
        else:
            self.shutter_status = 0
        
        
    def _interprete_GetStat8(self, reply):
        cleaned_reply = reply[2:]
        
        flag_byte_4 = int(cleaned_reply[:2],16)
        flag_byte_5 = int(cleaned_reply[2:4],16)
        
        internal_voltage = int(cleaned_reply[4:6],16)
        temperature1 = int(cleaned_reply[6:8],16)
        temperature2 = int(cleaned_reply[8:10],16)
        energy = int(cleaned_reply[10:14],16)
        quantity_counter = int(cleaned_reply[14:18],16)
        shot_counter_value = int(cleaned_reply[18:26],16)
        
        
        flag_bytes_4 = {0:"Static error", 1:"Laser head chamber open", 2:"External interlock circuit open (Remote)", 
                        3:"Temperature too high (> 60°C)", 4:"Temperature 1 too high (> 48°C) ",
                        5:"Temperature 2 too high (> 48°C) warning", 6:"Energy monitor error occurred"}
        
        flag_bytes_5 = {0:"Operation error: Laser must be switched off", 3:"High voltage supply error or temperature error",
                        4:"Temperature error 1", 5:"Temperature error 2", 6:"Power switch is damaged", 7:"Power supply is too weak"}
        
    def _interprete_GetSernum(self, reply):
        cleaned_reply = reply[2:]
        
        stepper_mode = int(cleaned_reply[:2],16)
        stepper_setpoint = int(cleaned_reply[2:6],16)
        actual_stepper_position = int(cleaned_reply[6:10],16)
        actual_transmission = int(cleaned_reply[10:12],16)/2
        
    def _interprete_GetAttenuatorStatus(self, reply):
        cleaned_reply = reply[2:]
        
        self.laser_serial_number = int(cleaned_reply[:8],16)
        self.energy_monitor_serial_number = int(cleaned_reply[8:12],16)
        
        
    def _interprete_GetEnergyValues(self, reply):
        cleaned_reply = reply[1:]
        
        stored_energy_value_count = int(cleaned_reply[:2],16)
        following_energy_values = int(cleaned_reply[2:4],16)
        
        
        for i in range(following_energy_values):
            energy = int(cleaned_reply[4+4*i:4+4*(i+1)],16) * 250/64000 # micro Joule
            
            self.energy_values.append(energy )
        
        
    def _interprete_GetVer3(self, reply):
        cleaned_reply = reply[2:]
        
        main_rev_byte = int(cleaned_reply[:2],16)
        release_byte = int(cleaned_reply[2:4],16)
        type_byte_1 = int(cleaned_reply[4:6],16)
        type_byte_2 = int(cleaned_reply[6:8],16)
        program_version = cleaned_reply[8:16],16
        following_chars = int(cleaned_reply[16:18],16)
        laser_type = cleaned_reply[18:18+following_chars]
        
        release_bytes = {0:"Shutter control is not supported", 1:"Attenuation module is supported",
                         3:"High voltage control is supported", 6:"Energy measuring is supported"}
        
        type_bytes_1 = {}
        
        """
        Needs to be understood
        """
        
        
        
    
class LaserCommunicationThread(QThread):
    
    
    recieved_command_signal = pyqtSignal()
    
    
    def __init__(self, main_window, com_port="/dev/ttyUSB0"):
        QThread.__init__(self)
        
        self.main_window = main_window
        self.recieved_command_signal.connect(self.main_window.display_laser_status)
        
        #serial connection parameters
        
        self.baud_rate = 9600
        self.parity = serial.PARITY_NONE
        self.bytesize = 8
        self.stopbits = 1
        self.rtscts = 1
        """
        self.serial_connection = serial.Serial(com_port, self.baud_rate, parity=self.parity,
                                               bytesize=self.bytesize, stopbits=self.stopbits,
                                              rtscts=self.rtscts)
        """
        self.handler = LaserCommunicationHandler()
        self.serial_connection = DummySerial(self.handler)
        
        self.command_queue = []
    
        self.alive = True
        self.recieved_messages = []
        self.outgoing_messages = []
        self.message_limit = 1000
        
        
    
    def run(self):
        
        print("Communication Thread started")
        
        while(self.alive):
            
            
            read_timeout_counter = 10
            while(self._waiting_bytes() != 0 and read_timeout_counter > 0):
                recieved_message = self.serial_connection.read_until(self.handler.end_delimiter)
                
                self.recieved_messages.append(recieved_message)
                
                if len(self.recieved_messages) > self.message_limit:
                    self.recieved_messages.pop(0)
                    
                read_timeout_counter -= 1
            
            if len(self.outgoing_messages) > 0:
                outgoing_message = self.outgoing_messages.pop(0)
                
                self.serial_connection.write(outgoing_message)
    
            time.sleep(0.005)
            self.recieved_command_signal.emit()
        
        
        print("Communication Thread ended")
    
    def _waiting_bytes(self):
        global python_version
        global serial_version
        
        incoming_byts_in_buffer = 0
            
        if python_version >= 3 and serial_version >=3:
            incoming_byts_in_buffer = self.serial_connection.in_waiting
        else:
            incoming_byts_in_buffer = self.serial_connection.inWaiting()
    
        return incoming_byts_in_buffer
    
    
    def execute_command(self, command_string, command_parameter=None):
        print("Executing: {}".format(command_string))
        request_string = self.handler.compose_command(command_string, command_parameter)
        
        self.outgoing_messages.append(request_string)
    
    def LaserOn(self):
        self.execute_command("LaserOn")
    
    def LaserOff(self):
        self.execute_command("LaserOff")
    
    def OpenShutter(self):
        self.execute_command("SetShutter", 1)
        self.handler.shutter_status = 1
        
    def CloseShutter(self):
        self.execute_command("SetShutter", 0)
        self.handler.shutter_status = 0
    
    def RepetitionOn(self):
        self.execute_command("RepetitionOn")
    
    def BurstOn(self):
        self.execute_command("BurstOn")
    
    def ExternalTriggerOn(self):
        self.execute_command("ExtTrigmode")
    
    def Stop(self):
        self.execute_command("LaserStop")
    
    def setRepetitionRate(self, frequency):
        self.execute_command("SetRepetitionFrequency", frequency)
    
    def setRepetitionQuantity(self, quantity):
        self.execute_command("SetBurstQuantity", quantity)
        
    
    def ToggleShutter(self):
        if self.handler.shutter_status == 0:
            self.OpenShutter()
        else:
            self.CloseShutter()
    

import datetime
class DummySerial(object):

    def __init__(self, handler):
        self.last_called = datetime.datetime.utcnow()
        self.buffer = ""
        self.handler = handler
    @property
    def in_waiting(self):
        s = "<@!UT040003000A14320000000088\r"
        
        delta_t = (datetime.datetime.utcnow() - self.last_called).total_seconds()
        if delta_t > 0.5:
            self.buffer += s
            self.last_called = datetime.datetime.utcnow()
        return len(self.buffer)
            
    def read_until(self, char):
        index = self.buffer.find(char)+1
        reply = self.buffer[:index]
        self.buffer = self.buffer[index:]
        return reply
    
    def write(self, out):
        print("Output:")
        print(out)
        

if __name__ == "__main__":
    
    t = LaserCommunicationThread()
    t.start()