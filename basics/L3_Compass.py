import L2_compass_heading as compheading
import L1_log as log
from time import sleep
import L1_ina as ina

while(1):
    heading = compheading.get_heading()
    log.tmpFile(heading, "tmpdegrees.txt")
    

    if(heading > 0 and heading <= 22.5):
        cardinal = "North"
    elif(heading > 22.5 and heading <= 67.5):
        cardinal = "North East"
    elif(heading > 67.5 and heading <= 112.5):
        cardinal = "East"
    elif(heading > 112.5 and heading <= 157.5):
        cardinal = "South East"
    elif(heading > 157.5 and heading <= 180):
        cardinal = "South"
    
    elif(heading < 0 and heading >= -22.5):
        cardinal = "North"
    elif(heading < -22.5 and heading >= -67.5):
        cardinal = "North West"
    elif(heading < -67.5 and heading >= -112.5):
        cardinal = "West"
    elif(heading < -112.5 and heading >= -157.5):
        cardinal = "South West" 
    elif(heading < -157.5 and heading >= -180):
        cardinal = "North"

    print(cardinal)

    log.stringTmpFile(cardinal, "tmpheading.txt")

    voltage = ina.readVolts()
    log.tmpFile(voltage, "tmp.txt")
    sleep(1)

