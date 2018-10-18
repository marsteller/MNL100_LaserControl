# -*- coding: utf-8 -*-
"""
Created on Tue Oct 16 00:51:44 2018

@author: manshiro
"""

import sys
from LedIndicatorWidget import *
import LaserControlMainWindow
import laser_communication
from PyQt5.QtWidgets import QApplication, QMainWindow, QErrorMessage

from PyQt5.QtCore import (QCoreApplication, QObject, QRunnable, QThread,
                          QThreadPool, pyqtSignal, pyqtSlot, Qt)
import time
import logging

class LaserControl(QMainWindow):
    
    set_repetition_rate_signal = pyqtSignal(int)
    set_repetition_quantity_signal = pyqtSignal(int)
    
    def __init__(self):
        super(self.__class__, self).__init__()
        logging.info("Initializing LaserControl GUI")

        self.ui = LaserControlMainWindow.Ui_MainWindow()

        self.ui.setupUi(self)
        
        
        logging.info("Looking for COM ports")
        for port in self.laser_communication_thread.available_comports:
            self.ui.comboBox.addItem(port.device)
        
        self.error_window = None
        
        self.ui.connection_label.setText("Connecting to laser...")
        try:
            self.laser_communication_thread = laser_communication.LaserCommunicationThread(self, debug=False)
        
        
            
            logging.info("Connecting GUI signals")
            self.ui.stop_button.clicked.connect(self.laser_communication_thread.Stop)
            self.ui.burst_on_button.clicked.connect(self.laser_communication_thread.BurstOn)
            self.ui.laser_off_button.clicked.connect(self.laser_communication_thread.LaserOff)
            self.ui.standby_button.clicked.connect(self.laser_communication_thread.LaserOn)
            self.ui.toggle_shutter_button.clicked.connect(self.laser_communication_thread.ToggleShutter)
            self.ui.external_trigger_on_button.clicked.connect(self.laser_communication_thread.ExternalTriggerOn)
            self.ui.repetition_on_button.clicked.connect(self.laser_communication_thread.RepetitionOn)
            self.ui.repetition_rate_spinBox.editingFinished.connect(self.repetition_rate_changed)
            self.set_repetition_rate_signal.connect(self.laser_communication_thread.setRepetitionRate)
            self.ui.repetition_quantity_spinBox.editingFinished.connect(self.repetition_quantity_changed)
            self.set_repetition_quantity_signal.connect(self.laser_communication_thread.setRepetitionQuantity)

            logging.info("Starting laser communication thread.")
            self.laser_communication_thread.start()
        except Exception as e:
            self.error_window = QErrorMessage(self)
            self.error_window.message = str(e)
        

    def closeEvent(self, event):
        try:
            logging.info("Stopping laser communication thread.")
            self.laser_communication_thread.alive = False
            
            while(self.laser_communication_thread.isFinished() == False):
                time.sleep(0.01)
            
            logging.info("Laser communication thread stopped.")
        except AttributeError:
            pass
        logging.info("Exiting now.")
        event.accept()
        
        
    def repetition_rate_changed(self):
        logging.debug("GUI: Chaning laser repetition rate to: {}".format(self.ui.repetition_rate_spinBox.value()))
        self.set_repetition_rate_signal.emit(self.ui.repetition_rate_spinBox.value())
        
    def repetition_quantity_changed(self):
        logging.debug("GUI: Chaning laser repetition quantity to: {}".format(self.ui.repetition_quantity_spinBox.value()))
        self.set_repetition_quantity_signal.emit(self.ui.repetition_quantity_spinBox.value())
        
        
    def display_laser_status(self):
        logging.debug("GUI: Upadting laser status displays")
        
        self.ui.shutter_status_led.setChecked(self.laser_communication_thread.handler.shutter_open)
        if self.laser_communication_thread.handler.shutter_open:
            self.ui.shutter_status_label.setText("Open")
        else:
            self.ui.shutter_status_label.setText("Closed")
            
        
        self.ui.burst_on_led.setChecked(self.laser_communication_thread.handler.burst_on)
        self.ui.repetition_on_led.setChecked(self.laser_communication_thread.handler.repetition_on)
        self.ui.laser_ready_led.setChecked(self.laser_communication_thread.handler.ready)
        self.ui.laser_on_led.setChecked(self.laser_communication_thread.handler.standby)
        self.ui.external_trigger_on_led.setChecked(self.laser_communication_thread.handler.external_trigger_on)
        
        self.ui.temperature_bar.setValue(self.laser_communication_thread.handler.temperature1)
        
        self.ui.repetition_quantity_spinBox.setValue(self.laser_communication_thread.handler.quantity)
        self.ui.repetition_rate_spinBox.setValue(self.laser_communication_thread.handler.frequency)
        self.ui.repetition_bar.setValue(self.laser_communication_thread.handler.quantity_counter)
        
        self.ui.total_shots_label.setText("Total shots:\n{}".format(self.laser_communication_thread.handler.shot_counter_value))

if __name__ == "__main__":
    
    logging.basicConfig(filename="laser_control_log.log", format='%(asctime)s %(message)s')
    app = QApplication(sys.argv)
    form = LaserControl()
    form.show()
    form.error_window.showMessage(form.error_window.message)
    
    sys.exit(app.exec_())
