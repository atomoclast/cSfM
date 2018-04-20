from math import tan

pxW = 4032
f_mm = 4.25 #mm
fov = 69.4 # deg

f_px = (pxW * 0.5) / tan(fov*0.5*(3.1415/180))

print "Focal length in px: ", f_px

