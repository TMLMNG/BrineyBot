#!/usr/bin/env python3
# coding: utf-8

import pump as pump
from time import sleep
from signal import pause

# Change GPIO Pin here
PUMP_GPIO_PIN = 2

# create our pump "object"
print('[INFO]: Create pump obj')
p = pump.setup(PUMP_GPIO_PIN)

# check the pin
print('[INFO]: Check pump pin')
pump_pin = pump.pin(p)
print('        Pump is set up to use pin: {pin}'.format(pin=pump_pin))

# prime the pump (seconds)
print('[INFO]: Prime pump')
PRIME_TIME = 10
pump.prime(p, PRIME_TIME)

# test on / off / toggle
print('[INFO]: Pump ON')
pump.on(p)
sleep(3)
print('[INFO]: Pump OFF')
pump.off(p)
sleep(3)
print('[INFO]: Pump Toggle (ON)')
pump.toggle(p)
sleep(3)
print('[INFO]: Pump Toggle (OFF)')
pump.toggle(p)
sleep(10)

# test pump states
print('[INFO]: Pump Toggle (ON)')
pump.toggle(p)
active = pump.is_active(p)
state = pump.state(p)
print('        Pump is active: {active}'.format(active=active))
print('        Pump is active: {state}'.format(state=state))
sleep(3)

print('[INFO]: Pump Toggle (OFF)')
pump.toggle(p)
active = pump.is_active(p)
state = pump.state(p)
print('        Pump is active: {active}'.format(active=active))
print('        Pump is active: {state}'.format(state=state))

print('[INFO]: Pump OFF')
pump.off(p)
