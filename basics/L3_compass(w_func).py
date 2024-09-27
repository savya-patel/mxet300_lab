import L2_compass_heading as compheading
import L1_log as log
from time import sleep
import L1_ina as ina

def cardinal_from_heading(heading):
    if(heading > 0 and heading <= 22.5):
        return "North"
    elif(heading > 22.5 and heading <= 67.5):
        return "North East"
    elif(heading > 67.5 and heading <= 112.5):
        return "East"
    elif(heading > 112.5 and heading <= 157.5):
        return "South East"
    elif(heading > 157.5 and heading <= 180):
        return "South"
    elif(heading < 0 and heading >= -22.5):
        return "North"
    elif(heading < -22.5 and heading >= -67.5):
        return "North West"
    elif(heading < -67.5 and heading >= -112.5):
        return "West"
    elif(heading < -112.5 and heading >= -157.5):
        return "South West" 
    elif(heading < -157.5 and heading >= -180):
        return "South"

while(1):
    heading = compheading.get_heading()
    log.tmpFile(heading, "tmpdegrees.txt")

    cardinal = cardinal_from_heading(heading)
    log.stringTmpFile(cardinal, "tmpheading.txt")

    voltage = ina.readVolts()
    log.tmpFile(voltage, "tmp.txt")
    sleep(1)
