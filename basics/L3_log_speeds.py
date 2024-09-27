import L1_log as log
import L2_kinematics as k
from time import sleep


while(1):
    pds = k.getPdCurrent()
    chass = k.getMotion()
    print("PDL: ", pds[0], "\t PDR:", pds[1])
    print("FWD Velocity (m/s): ", chass[0], "\t Angular Velocity (rad/s):", chass[1])
    log.tmpFile(pds[0], "pdl.txt")
    log.tmpFile(pds[1], "pdr.txt")
    log.tmpFile(chass[0], "fwd.txt")
    log.tmpFile(chass[1], "ang.txt")
    sleep(1)