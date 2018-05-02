#!/usr/bin/env python

from math import tan
from sys import argv, exit

# pxW = 4032
# f_mm = 4.25 #mm
# fov = 69.4 # deg

def calculate_fpx(pxW, f_mm, fov):

	f_px = (pxW * 0.5) / tan(fov*0.5*(3.1415/180))

	print "Focal length in px: ", f_px

if __name__ == "__main__":

	if(len(argv) != 4):
		print "Usage: calculate_focal_length.py <pxW> <f_mm> <fov>"
		exit()

	pxW = float(argv[1])
	f_mm = float(argv[2])
	fov = float(argv[3])

	calculate_fpx(pxW, f_mm, fov)
