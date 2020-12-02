#!/usr/bin/env python3
# coding: utf-8

from gpiozero import LED
from time import sleep

# Set GPIO pin to turn on the pump
# returns pump "object"
def setup(PUMP_GPIO_PIN):
    return LED(PUMP_GPIO_PIN)

# Prime the pump to make sure the tubing is filled with liquid
# before we start operating (determine time experimentally)
def prime(pump, duration):
    pump.on()
    sleep(duration)
    pump.off()

# Turns the device on
def on(pump):
    pump.on()

# Turns the device off
def off(pump):
    pump.off()

# Reverse the state of the device
def toggle(pump):
    pump.toggle()

# Returns True if the device is currently active and False otherwise
# this is always a boolean
def is_active(pump):
    return pump.is_lit

# The Pin that the device is connected to.
# This will be None if the device has been closed
def pin(pump):
    return pump.pin

# Returns 1 if the device is currently active and 0 otherwise
def state(pump):
    return pump.value
