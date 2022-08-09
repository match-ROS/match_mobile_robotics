#!/usr/bin/env python3
import signal
import sys
import time
import os
import subprocess



output_string = subprocess.Popen(["./ckserver", "10.145.8.32", "4567"], 
                          stdout=subprocess.PIPE).communicate()[0].decode("utf-8")

print(output_string)

