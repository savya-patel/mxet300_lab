import L1_log as log
import L2_vector as vector
import L1_ina as ina
from time import sleep


while(1):
    vec = vector.getNearest()
    print("m: ", vec[0], "\t angle:", vec[1])
    log.tmpFile(vec[0], "distance.txt")
    log.tmpFile(vec[1], "angle.txt")
    cartesian = vector.polar2cart(vec[0],vec[1])
    log.tmpFile(cartesian[0], "x_value.txt")
    log.tmpFile(cartesian[1], "y_value.txt")    
    voltage = ina.readVolts()
    log.tmpFile(voltage, "tmp.txt")
    sleep(1)
    sleep(1)