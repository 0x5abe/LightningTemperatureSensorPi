#!/usr/bin/env python

# read_PWM.py
# 2015-12-08
# Public Domain

import time
import pigpio # http://abyz.co.uk/rpi/pigpio/python.html
import sys
import pika
import json
import datetime
sys.path.append('../')
import time
from DFRobot_AS3935_Lib import DFRobot_AS3935
from BMP085_Lib import BMP085
import RPi.GPIO as GPIO
from datetime import datetime

#I2C address
AS3935_I2C_ADDR1 = 0X01
AS3935_I2C_ADDR2 = 0X02
AS3935_I2C_ADDR3 = 0X03

#Antenna tuning capcitance (must be integer multiple of 8, 8 - 120 pf)
AS3935_CAPACITANCE = 8
IRQ_PIN = 10

GPIO.setmode(GPIO.BOARD)

lightningSensor = DFRobot_AS3935(AS3935_I2C_ADDR2, bus = 1)
if (lightningSensor.reset()):
  print("init lightningSensor sucess.")
else:
  print("init lightningSensor fail")
  while True:
    pass
#Configure lightningSensor
lightningSensor.power_up()

#set indoors or outdoors models
#lightningSensor.set_indoors()
lightningSensor.set_outdoors()

#disturber detection
lightningSensor.disturber_en()
#lightningSensor.disturber_dis()

#lightningSensor.set_irq_output_source(0)
time.sleep(0.5)
#set capacitance
lightningSensor.set_tuning_caps(AS3935_CAPACITANCE)

# Connect the IRQ and GND pin to the oscilloscope.
# uncomment the following sentences to fine tune the antenna for better performance.
# This will dispaly the antenna's resonance frequency/16 on IRQ pin (The resonance frequency will be divided by 16 on this pin)
# Tuning AS3935_CAPACITANCE to make the frequency within 500/16 kHz plus 3.5% to 500/16 kHz minus 3.5%
#
lightningSensor.set_lco_fdiv(3)
lightningSensor.set_irq_output_source(3)

#Set the noise level,use a default value greater than 7
lightningSensor.set_noise_floor_lv1(2)
#noiseLv = lightningSensor.get_noise_floor_lv1()

#used to modify WDTH,alues should only be between 0x00 and 0x0F (0 and 7)
lightningSensor.set_watchdog_threshold(2)
#wtdgThreshold = lightningSensor.get_watchdog_threshold()

#used to modify SREJ (spike rejection),values should only be between 0x00 and 0x0F (0 and 7)
lightningSensor.set_spike_rejection(2)
#spikeRejection = lightningSensor.get_spike_rejection()

#view all register data
#lightningSensor.print_all_regs()

class reader:
   """
   A class to read PWM pulses and calculate their frequency
   and duty cycle.  The frequency is how often the pulse
   happens per second.  The duty cycle is the percentage of
   pulse high time per cycle.
   """
   def __init__(self, pi, gpio, weighting=0.0):
      """
      Instantiate with the Pi and gpio of the PWM signal
      to monitor.

      Optionally a weighting may be specified.  This is a number
      between 0 and 1 and indicates how much the old reading
      affects the new reading.  It defaults to 0 which means
      the old reading has no effect.  This may be used to
      smooth the data.
      """
      self.pi = pi
      self.gpio = gpio

      if weighting < 0.0:
         weighting = 0.0
      elif weighting > 0.99:
         weighting = 0.99

      self._new = 1.0 - weighting # Weighting for new reading.
      self._old = weighting       # Weighting for old reading.

      self._high_tick = None
      self._period = None
      self._high = None

      pi.set_mode(gpio, pigpio.INPUT)

      self._cb = pi.callback(gpio, pigpio.EITHER_EDGE, self._cbf)

   def _cbf(self, gpio, level, tick):

      if level == 1:

         if self._high_tick is not None:
            t = pigpio.tickDiff(self._high_tick, tick)

            if self._period is not None:
               self._period = (self._old * self._period) + (self._new * t)
            else:
               self._period = t

         self._high_tick = tick

      elif level == 0:

         if self._high_tick is not None:
            t = pigpio.tickDiff(self._high_tick, tick)

            if self._high is not None:
               self._high = (self._old * self._high) + (self._new * t)
            else:
               self._high = t

   def frequency(self):
      """
      Returns the PWM frequency.
      """
      if self._period is not None:
         return 1000000.0 / self._period
      else:
         return 0.0

   def pulse_width(self):
      """
      Returns the PWM pulse width in microseconds.
      """
      if self._high is not None:
         return self._high
      else:
         return 0.0

   def duty_cycle(self):
      """
      Returns the PWM duty cycle percentage.
      """
      if self._high is not None:
         return 100.0 * self._high / self._period
      else:
         return 0.0

   def cancel(self):
      """
      Cancels the reader and releases resources.
      """
      self._cb.cancel()

if __name__ == "__main__":

   import time
   import pigpio
   import read_PWM

   PWM_GPIO = 10
   RUN_TIME = 60.0
   SAMPLE_TIME = 2.0

   pi = pigpio.pi()

   p = read_PWM.reader(pi, PWM_GPIO)

   start = time.time()

   while (time.time() - start) < RUN_TIME:

      time.sleep(SAMPLE_TIME)

      f = p.frequency()
      pw = p.pulse_width()
      dc = p.duty_cycle()
     
      print("f={:.1f} pw={} dc={:.2f}".format(f, int(pw+0.5), dc))

   p.cancel()

   pi.stop()

