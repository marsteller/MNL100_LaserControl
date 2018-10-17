# -*- coding: utf-8 -*-
"""
Created on Mon Oct 15 16:40:11 2018

@author: Alexander Marsteller
"""

import serial
import serial.tools.list_ports
from PyQt5.QtCore import (QCoreApplication, QObject, QRunnable, QThread,
                          QThreadPool, pyqtSignal, pyqtSlot)
import sys
import time
import numpy as np

python_version = float(sys.version_info.major)
serial_version = float(serial.__version__)

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
        
        
        self.flag_byte_1 = 4
        #flag_byte_2 = reply[2:4]   # unused
        self.flag_byte_3 = 0
        self.quantity = 0
        self.frequency = 0
        self.hv = 0
        self.energy = 0
        self.flag_byte_4 = 0
        self.flag_byte_5 = 0
        
        self.internal_voltage = 0
        self.temperature1 = 0
        self.temperature2 = 0
        self.energy = 0
        self.quantity_counter = 0
        self.shot_counter_value = 0
        self.stepper_mode = 0
        self.stepper_setpoint = 0
        self.actual_stepper_position = 0
        self.actual_transmission = 0
        
        self.flag_bytes_1 = {0:"Shutter is Open", 2:"Laser is Ready for Operation", 3:"Laser Standby", 
                      4:"LaserMode: Off", 5:"LaserMode: Repetition", 6:"LaserMode: Burst", 7:"LaserMode: External triggering"}
        self.flag_bytes_3 = {0:"Service Mode activated", 5:"EEPROM error", 6:"Watchdog Reset occurred"}
        
        
        self.flag_bytes_4 = {0:"Static error", 1:"Laser head chamber open", 2:"External interlock circuit open (Remote)", 
                        3:"Temperature too high (> 60°C)", 4:"Temperature 1 too high (> 48°C) ",
                        5:"Temperature 2 too high (> 48°C) warning", 6:"Energy monitor error occurred"}
        
        self.flag_bytes_5 = {0:"Operation error: Laser must be switched off", 3:"High voltage supply error or temperature error",
                        4:"Temperature error 1", 5:"Temperature error 2", 6:"Power switch is damaged", 7:"Power supply is too weak"}
        
        self.release_bytes = {0:"Shutter control is not supported", 1:"Attenuation module is supported",
                         3:"High voltage control is supported", 6:"Energy measuring is supported"}
        
        self.type_bytes_1 = {}
        
        self.energy_values = []
        
        
        self.shutter_status = 0
        
        
        
        
        
    def _calculate_frame_check_squence(self, telegram):
        
        encoded_telegram = telegram.encode("ASCII")
        fcs = hex(sum(bytearray(encoded_telegram)) % 256).upper()[2:]
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
        print(reply)
        print(reply_fcs)
        print(cleaned_reply[:-2])
        
        calculated_reply_fcs = self._calculate_frame_check_squence(cleaned_reply[:-2])
        print(calculated_reply_fcs)
        
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
            
            return response_type
        
    def _interprete_GetShortStatus(self, reply):
        cleaned_reply = reply[1:]
        
        response_dict = {"00":"Standby", "01":"Laser is Working", "03":"EEPROM Error",
                         "04":"Energy Monitor Error occured", "05":"Temperature too high (>48°C)",
                         "06":"Static Error", "07":"Operation Error: Laser mus be switched off"}
        
        return response_dict[cleaned_reply]
    
    def _interprete_GetStat7(self, reply):
        cleaned_reply = reply[2:]
        
        self.flag_byte_1 = int(cleaned_reply[:2],16)
        #flag_byte_2 = reply[2:4]   # unused
        self.flag_byte_3 = int(cleaned_reply[4:6],16)
        self.quantity = int(cleaned_reply[6:10],16)
        self.frequency = int(cleaned_reply[10:12],16)
        self.hv = int(cleaned_reply[12:14],16)
        self.energy = int(cleaned_reply[18:22],16)
        
        if self.flag_byte_1 == 0:
            self.shutter_status = 1
        else:
            self.shutter_status = 0
        
        
    def _interprete_GetStat8(self, reply):
        cleaned_reply = reply[2:]
        
        self.flag_byte_4 = int(cleaned_reply[:2],16)
        self.flag_byte_5 = int(cleaned_reply[2:4],16)
        
        self.internal_voltage = int(cleaned_reply[4:6],16)
        self.temperature1 = int(cleaned_reply[6:8],16)
        self.temperature2 = int(cleaned_reply[8:10],16)
        self.energy = int(cleaned_reply[10:14],16)
        self.quantity_counter = int(cleaned_reply[14:18],16)
        self.shot_counter_value = int(cleaned_reply[18:26],16)
        
        print(self.internal_voltage)
        print(self.temperature1)
        print(self.temperature2)
        
    def _interprete_GetSernum(self, reply):
        cleaned_reply = reply[2:]
        
        self.stepper_mode = int(cleaned_reply[:2],16)
        self.stepper_setpoint = int(cleaned_reply[2:6],16)
        self.actual_stepper_position = int(cleaned_reply[6:10],16)
        self.actual_transmission = int(cleaned_reply[10:12],16)/2
        
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
        
        self.main_rev_byte = int(cleaned_reply[:2],16)
        self.release_byte = int(cleaned_reply[2:4],16)
        self.type_byte_1 = int(cleaned_reply[4:6],16)
        self.type_byte_2 = int(cleaned_reply[6:8],16)
        self.program_version = cleaned_reply[8:16],16
        following_chars = int(cleaned_reply[16:18],16)
        self.laser_type = cleaned_reply[18:18+following_chars]
        
        
    
class LaserCommunicationThread(QThread):
    
    
    recieved_reply_signal = pyqtSignal(str)
    update_main_window_signal = pyqtSignal()
    
    
    def __init__(self, main_window, com_port="/dev/ttyUSB0"):
        QThread.__init__(self)
        self.available_comports = list(serial.tools.list_ports.comports())
        
        
        self.main_window = main_window
        self.recieved_reply_signal.connect(self.process_recieved_message)
        self.update_main_window_signal.connect(self.main_window.display_laser_status)
        
        self.handler = LaserCommunicationHandler()
        #serial connection parameters
        
        self.baud_rate = 9600
        self.parity = serial.PARITY_NONE
        self.bytesize = 8
        self.stopbits = 1
        self.rtscts = 1
        self.write_timeout = 5
        self.timeout = 0.2
        
        self.used_com_port = None
        if com_port != None:
            print("Using specified com port: {}".format(com_port))
            
            self.serial_connection = serial.Serial(com_port, self.baud_rate, parity=self.parity, 
                                                       bytesize=self.bytesize, stopbits=self.stopbits,
                                                       rtscts=self.rtscts, timeout=self.timeout, write_timeout=self.write_timeout)
            
            self.used_com_port = com_port

        else:
            print("No com port specified, trying to detect Laser...")
            for port in self.available_comports:
                print("Now trying com port: {}".format(port.device))
                try:
                    self.serial_connection = serial.Serial(port.device, self.baud_rate, parity=self.parity, 
                                                           bytesize=self.bytesize, stopbits=self.stopbits,
                                                           rtscts=self.rtscts, timeout=self.timeout, write_timeout=self.write_timeout)
                    
                    time.sleep(0.050)
                    self.serial_connection.reset_input_buffer()
                    outgoing_message = self.handler.compose_command("GetShortStatus").encode("ASCII")
                    self.serial_connection.write(outgoing_message)
                    time.sleep(0.010)
                    
                    if self._waiting_bytes() != 0:
                        recieved_message = self.serial_connection.read_until(self.handler.end_delimiter)
                        
                        if "<@!W" in recieved_message:
                            self.used_com_port = port.device
                            print("Laser found on com port: {}".format(port.device))
                        
                            break
                        else:
                            self.serial_connection.close()
                            print("Nothing found on com port: {}".format(port.device))
                            time.sleep(0.05)
                    else:
                        self.serial_connection.close()
                        print("Nothing found on com port: {}".format(port.device))
                        time.sleep(0.05)
                        
                except Exception as e:
                    print(e)
                    try:
                        self.serial_connection.close()
                    except Exception as e2:
                            print(e2)
        if self.used_com_port == None:
            raise serial.SerialException("Laser not found over serial interface") 
        
        #self.serial_connection = DummySerial(self.handler)
        
        self.command_queue = []
    
        self.alive = True
        self.recieved_messages = []
        self.outgoing_messages = []
        self.message_limit = 1000
        self.status_poll_interval = 1.0

        outgoing_message = "#!@gEB\r"
        self.serial_connection.write(outgoing_message.encode("ASCII"))
	
        
    
    def run(self):
        
        print("Communication Thread started")
        time.sleep(2.0)
        
        self.last_status_poll_time = datetime.datetime.utcnow()
        
        while(self.alive):
            read_timeout_counter = 10
            
            while(self._waiting_bytes() != 0 and read_timeout_counter > 0):
                recieved_message = self.serial_connection.read_until(self.handler.end_delimiter)
                print(recieved_message)
                recieved_message = recieved_message.decode("ASCII")
                messages = recieved_message.split("\r")
                for m in messages:
                        if m == "":
                               continue
                        self.recieved_messages.append(m)
                        self.recieved_reply_signal.emit(m)

                        if len(self.recieved_messages) > self.message_limit:
                               self.recieved_messages.pop(0)
                    
                read_timeout_counter -= 1
            
            if len(self.outgoing_messages) > 0:
                outgoing_message = self.outgoing_messages.pop(0)
                
                self.serial_connection.write(outgoing_message.encode("ASCII"))
                print("Writing: {}".format(outgoing_message))
    
            time.sleep(0.020)
            
            now = datetime.datetime.utcnow()
            if (now - self.last_status_poll_time).total_seconds() > self.status_poll_interval:
                self.last_status_poll_time = now
                self.QueryStatus()
        
        
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
    
    def process_recieved_message(self, message):
        respone_type = self.handler._interprete_response(message)
        
        print("Recieved:")
        print(respone_type)
        self.update_main_window_signal.emit()
    
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
        
    def CloseShutter(self):
        self.execute_command("SetShutter", 0)
    
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
            
    def QueryStatus(self):
        self.execute_command("GetShortStatus")
        self.execute_command("GetStat7")
        self.execute_command("GetStat8")
        
    def QueryShortStatus(self):
        self.execute_command("GetShortStatus")

import datetime
class DummySerial(object):

    def __init__(self, handler):
        self.last_called = datetime.datetime.utcnow()
        self.buffer = ""
        self.handler = handler
        self.shutter_status = 0
    @property
    def in_waiting(self):
        """
        s = "<@!UT040003000A14320000000088\r"
        
        delta_t = (datetime.datetime.utcnow() - self.last_called).total_seconds()
        if delta_t > 0.5:
            self.buffer += s
            self.last_called = datetime.datetime.utcnow()
        """
        return len(self.buffer)
            
    def read_until(self, char):
        index = self.buffer.find(char)+1
        reply = self.buffer[:index]
        self.buffer = self.buffer[index:]
        print("Reply:")
        print(reply)
        return reply.encode("ASCII")
    
    def write(self, out):
        print("Output:")
        print(out.decode("ASCII"))
        out = out.decode("ASCII")
        
        if out == self.handler.compose_command("SetShutter", 1):
            self.shutter_status = 1
        if out == self.handler.compose_command("SetShutter", 0):
            self.shutter_status = 0
        
        if out == self.handler.compose_command("GetStat7"):
            if self.shutter_status == 0:
                self.buffer += "<@!UT040003000A14320000000088\r"
            elif self.shutter_status == 1:
                self.buffer += "<@!UT000003000A14320000000084\r"
        
        if out == self.handler.compose_command("GetStat8"):
            temperature = 24 + np.random.random() * 20
            print(int(temperature))
            print(hex(int(temperature)))
            message = "<@!UU000000{}22000000000001154C".format(hex(int(temperature))[2:].upper())
            fcs = self.handler._calculate_frame_check_squence(message)
            self.buffer += message + fcs +"\r"
        

if __name__ == "__main__":
    handler = LaserCommunicationHandler()
    #serial connection parameters
        
    baud_rate = 9600
    parity = serial.PARITY_NONE
    bytesize = 8
    stopbits = 1
    rtscts = 1
    write_timeout = 5
    timeout = 0.2
        
    used_com_port = None
        
    serial_connection = serial.Serial("/dev/ttyUSB0", baud_rate, parity=parity, 
                                                   bytesize=bytesize, stopbits=stopbits,
                                                   rtscts=rtscts, timeout=timeout, write_timeout=write_timeout)
        
    outgoing_messages = []

    #serial_connection.write(handler.compose_command("LaserOn").encode("ASCII"))
    #time.sleep(2)
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8"))    

    outgoing_messages.append(handler.compose_command("LaserOn"))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8"))    

    outgoing_messages.append(handler.compose_command("SetShutter", 1))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8"))    
    outgoing_messages.append(handler.compose_command("SetShutter", 0))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8")) 

    outgoing_messages.append(handler.compose_command("RepetitionOn"))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8"))    

    outgoing_messages.append(handler.compose_command("SetShutter", 1))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8"))    
    outgoing_messages.append(handler.compose_command("SetShutter", 0))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8")) 

    outgoing_messages.append(handler.compose_command("LaserStop"))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8"))  

    outgoing_messages.append(handler.compose_command("BurstOn"))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8"))    

    outgoing_messages.append(handler.compose_command("SetShutter", 1))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8"))    
    outgoing_messages.append(handler.compose_command("SetShutter", 0))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8")) 

    outgoing_messages.append(handler.compose_command("LaserStop"))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8"))     
 
    outgoing_messages.append(handler.compose_command("LaserOff"))
    outgoing_messages.append(handler.compose_command("GetShortStatus"))
    outgoing_messages.append(handler.compose_command("GetStat7"))
    outgoing_messages.append(handler.compose_command("GetStat8"))    
   

    with open("communications_log.txt", "w") as outfile:
        for outgoing_message in outgoing_messages:
            outfile.write("-"*20 + "\n")
            outfile.write("Sending command:\n")
            outfile.write(outgoing_message)
            serial_connection.write(outgoing_message.encode("ASCII"))
            time.sleep(0.5)
            reply = serial_connection.read_until("\r")
            outfile.write("Recieved Reply:\n")            
            outfile.write(reply.decode("ASCII")+ "\n")
            print(reply)



	
