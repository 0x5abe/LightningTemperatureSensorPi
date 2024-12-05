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
AS3935_CAPACITANCE = 96
IRQ_PIN = 37

GPIO.setmode(GPIO.BOARD)

temperaturePressureSensor = BMP085()

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

lightningSensor.set_irq_output_source(0)
time.sleep(0.5)
#set capacitance
lightningSensor.set_tuning_caps(AS3935_CAPACITANCE)

# Connect the IRQ and GND pin to the oscilloscope.
# uncomment the following sentences to fine tune the antenna for better performance.
# This will dispaly the antenna's resonance frequency/16 on IRQ pin (The resonance frequency will be divided by 16 on this pin)
# Tuning AS3935_CAPACITANCE to make the frequency within 500/16 kHz plus 3.5% to 500/16 kHz minus 3.5%
#
# lightningSensor.setLco_fdiv(0)
# lightningSensor.setIrq_output_source(3)

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

LIGHTNING_QUEUE = 'lightning_data'
TEMP_PRESS_QUEUE = 'temperature_pressure_data'
HOST = '192.168.1.3'

#setup rabbitmq message queue
connection = pika.BlockingConnection(pika.ConnectionParameters(host=HOST, heartbeat=300, blocked_connection_timeout=600))
global lightningChannel
lightningChannel = connection.channel()
lightningChannel.queue_declare(queue=LIGHTNING_QUEUE)
global temperaturePressureChannel
temperaturePressureChannel = connection.channel()
temperaturePressureChannel.queue_declare(queue=TEMP_PRESS_QUEUE)

def callback_handle(channel):
  global lightningSensor
  time.sleep(0.005)
  intSrc = lightningSensor.get_interrupt_src()
  if intSrc == 1:
    lightning_distKm = lightningSensor.get_lightning_distKm()
    lightning_energy_val = lightningSensor.get_strike_energy_raw()
    print('Lightning occurs! - Distance: {} - Intensity: {} - Time: {}'.format(lightning_distKm, lightning_energy_val, datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")))    
    #print('Distance: %dkm'%lightning_distKm)
    #print('Intensity: %d '%lightning_energy_val)
    
    message = {
      "message": 'Lightning occurs!',
      "distance": lightning_distKm,
      "intensity": lightning_energy_val,
      "datetime": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
    }
    lightningChannel.basic_publish(exchange='', routing_key=LIGHTNING_QUEUE, body=json.dumps(message))
  elif intSrc == 2:
    print('Disturber discovered! - Time: {}'.format(datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")))
    message = {
      "message": 'Disturber discovered!',
      "datetime": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
    }
    lightningChannel.basic_publish(exchange='', routing_key=LIGHTNING_QUEUE, body=json.dumps(message))
  elif intSrc == 3:
    print('Noise level too high!')
    message = {
      "message": 'Noise level too high!',
      "datetime": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
    }
    lightningChannel.basic_publish(exchange='', routing_key=LIGHTNING_QUEUE, body=json.dumps(message))
  else:
    pass
#Set to input mode

GPIO.setup(IRQ_PIN, GPIO.IN)
#Set the interrupt pin, the interrupt function, rising along the trigger
GPIO.add_event_detect(IRQ_PIN, GPIO.RISING, callback = callback_handle)
print("start lightning detect.")

while True:
  connection.process_data_events()
  temp = '{0:0.2f} *C'.format(temperaturePressureSensor.read_temperature())
  presshPa = temperaturePressureSensor.read_pressure() * 0.01
  press = '{0:0.2f} hPa'.format(presshPa)
  alt = '{0:0.2f} m'.format(temperaturePressureSensor.read_altitude())
  slpresshPa = temperaturePressureSensor.read_sealevel_pressure() * 0.01
  slpress = '{0:0.2f} hPa'.format(slpresshPa)
  message = {
    "message": 'Temperature/Pressure data',
    "temperature": temp,
    "pressure": press,
    "altitude": alt,
    "sea_level_pressure": slpress, 
    "datetime": datetime.now().strftime("%Y-%m-%dT%H:%M:%S.%f")
  }
  temperaturePressureChannel.basic_publish(exchange='', routing_key=TEMP_PRESS_QUEUE, body=json.dumps(message))
  time.sleep(1.0)


