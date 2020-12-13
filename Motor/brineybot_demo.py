#!/usr/bin/env python3
# coding: utf-8

# imports
import CarRun2 as m
import pump as p
from time import sleep

# init
m.motor_init()
p.init(25)

# prime the pump (seconds)
print('[INFO]: Prime pump...')
PRIME_TIME = 10
p.prime(PRIME_TIME)
print('[INFO]: Pump OFF')
p.off()
sleep(3)

# Move Around ##################################
print('[INFO]: Move NORTH')
m.run(1) # north
sleep(1)

print('[INFO]: Pump Toggle (ON)')
p.toggle() # pump on
sleep(1)

print('[INFO]: Face WEST')
m.spin_left(1) # face west 
sleep(1)

print('[INFO]: Move West')
m.run(1) # west
sleep(1)

print('[INFO]: Face North')
m.spin_right(1) # face north
sleep(1)

print('[INFO]: Move NORTH')
m.run(1) # north
sleep(1)

print('[INFO]: Face East')
m.spin_right(1) # face east
sleep(1)

print('[INFO]: Move East')
m.run(1) # east
sleep(1)

print('[INFO]: Face South')
m.spin_right(1) # face south
sleep(1)

print('[INFO]: Move South')
m.run(1) # south
sleep(1)

# stop
print('[INFO]: Halting...')
m.brake(1)
p.toggle()
p.cleanup()
