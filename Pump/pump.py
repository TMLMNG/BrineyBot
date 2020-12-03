#!/usr/bin/env python3
# coding: utf-8

# imports
import RPi.GPIO as GPIO
import time

# declare pump pin
PUMP_PIN = 2

#Set the GPIO port to BCM encoding mode.
GPIO.setmode(GPIO.BCM)

#Ignore warning information
GPIO.setwarnings(False)

# Set GPIO pin to turn on the pump
def init(PUMP_PIN):
    GPIO.setup(PUMP_PIN, GPIO.OUT)

# Prime the pump to make sure the tubing is filled with liquid
# before we start operating (determine time experimentally)
def prime(duration):
    GPIO.output(PUMP_PIN, GPIO.HIGH)
    time.sleep(duration)
    GPIO.output(PUMP_PIN, GPIO.LOW)

# Turns the device on
def on():
    GPIO.output(PUMP_PIN, GPIO.HIGH)

# Turns the device off
def off():
    GPIO.output(PUMP_PIN, GPIO.LOW)

# Reverse the state of the device
def toggle():
    GPIO.output(PUMP_PIN, not GPIO.input(PUMP_PIN))

# Returns True if the device is currently active and False otherwise
# this is always a boolean
def is_active():
    return GPIO.input(PUMP_PIN) == 1

# The Pin that the device is connected to.
# This will be None if the device has been closed
def pin():
    return PUMP_PIN

# Returns 1 if the device is currently active and 0 otherwise
def state():
    return GPIO.input(PUMP_PIN)

# cleanup
def cleanup():
    GPIO.cleanup()
