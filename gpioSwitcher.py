#!/usr/bin/env python

import subprocess 
import sys
import os
import time

class gpioSwitcher:
    def __init__(self):
        self.SELECT_GPIO = "21"
        self.ENABLE_GPIO = "19"

    # GPIO direction set
    def setGPIODir(self, gpio, direction):
        dir_fd = open("/sys/class/gpio/gpio"+str(gpio)+"/direction", "w")
        dir_fd.write(direction)
        dir_fd.close()

    # Open the GPIO
    def openGPIO(self, gpio):
        # Check and see if the GPIO is already exported
        if not os.path.isdir("/sys/class/gpio/gpio"+str(gpio)):
            exp_fd = open("/sys/class/gpio/export", "w")
            exp_fd.write(gpio)
            exp_fd.close()

        self.setGPIODir(gpio, "out");

    def closeGPIO(self, gpio):
        unexp_fd = open("/sys/class/gpio/unexport", "w")
        unexp_fd.write(gpio)
        unexp_fd.close()

    def setGPIO(self, gpio, value):
        val_fd = open("/sys/class/gpio/gpio"+str(gpio)+"/value", "w")
        val_fd.write(value)
        val_fd.close()

    def openSetClose(self, gpio, value):
        self.openGPIO(gpio)
        self.setGPIO(gpio, value)
        self.closeGPIO(gpio)

    def disconnectAndExit(self):
        self.openSetClose(self.SELECT_GPIO, "0")
        self.openSetClose(self.ENABLE_GPIO, "1")

    def start(self):
        self.openSetClose(self.SELECT_GPIO, "0")
        self.openSetClose(self.ENABLE_GPIO, "0")
        time.sleep(0.5)

if __name__ == "__main__":

    g = gpioSwitcher()
    
    g.enableGPIO()
    time.sleep(0.2)
    g.returnGPIO()

    gpioState = False

    print "Type 'on' to open gimbal USB, 'off' to disable, 'exit' to disconnect and exit."

    while True:
        time.sleep(0.2)

        print "GPIO set to gimbal port: ",gpioState

        key = raw_input()

        if key=="on":
            g.enableGPIO()
            gpioState = True

        if key=="off":
            g.returnGPIO()
            gpioState = False

        if key=="exit":
            g.returnGPIO()
            print "Reset!"
            break

