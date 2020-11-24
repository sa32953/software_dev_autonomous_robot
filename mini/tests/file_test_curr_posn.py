
import subprocess
import time

subprocess.call(['./b_test_curr_posn.sh'])
time.sleep(3)
subprocess.call(['./b_get_laser_scan.sh'])