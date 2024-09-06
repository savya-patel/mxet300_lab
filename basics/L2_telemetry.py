import L1_ina as ina
import L1_log as log
from time import sleep

while(1):
    voltage = ina.readVolts()
    log.tmpFile(voltage, "tmp.txt")
    sleep(1)

