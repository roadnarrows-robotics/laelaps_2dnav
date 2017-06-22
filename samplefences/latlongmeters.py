#! /usr/bin/env python

################################################################################
# 
# File: latlongmeters.py
#
# Usage: latlongmeters.py [jsonfile]
#         latlongmeters.py --help
#
# Description:
#   If a Json file is specified, the file is parsed. Otherwise, interactive
#   mode is entered to gather the data. The output is a list of geographic
#   positions in longitude, latitude order. The output is compatible
#   with Geofrenzy Feature Collections 'coordinates' field Json files.
#
# See Also:
#   The shape files triangle.json, ..., tee.json serve as examples.
#
################################################################################

import sys
import os
import getopt
import json
import math as m
import numpy as np

# Radius of earth in KM
Re = 6378.137

#
# Some cool geographical coordinates in degress latitude and longitude.
#
SomeGeoCoords = {
  'Berlin':       (52.5200, 13.4050),
  'Boulder':      (40.38085964, -105.099197),
  'Brisbane':     (-27.4698, 153.0251),
  'Fiji':         (-17.7134, 178.0650),
  'Guam':         (13.4443, 144.7937),
  'New York':     (40.7128, -74.0059),
  'Las Vegas':    (36.1699, -115.1398),
  'Los Angeles':  (34.0522, -118.2437)
}

Input = {
    'starting_longitude': 0.0,
    'starting_latitude': 0.0,
    'scale': 1.0,
    'translate': (0.0, 0.0),
    'rotate': 0.0,
    'path': []
}

def radians(deg):
  return m.pi * deg / 180.0

def degrees(rad):
  return 180.0 * rad / m.pi

##
# \brief Measure distance between two lat,lon positions.
#
# Simplified "Earth is a ball" measurement.
#
# \param lat1   Source geographic latitude (degrees)
# \param lon1   Source geographic longitude (degrees)
# \param lat2   Destination geographic latitude (degrees)
# \param lon2   Destination geographic longitude (degrees)
#
# \return Returns distance in meters.
#
def geomeasure(lat1, lon1, lat2, lon2):
  rlat1 = radians(lat1)
  rlon1 = radians(lon1)
  rlat2 = radians(lat1)
  rlon2 = radians(lon1)

  dLat = rlat2 - rlat1
  dLon = rlon2 - rlon1

  a = m.sin(dLat/2) * m.sin(dLat/2) + \
    m.cos(rlat1) * m.cos(rlat2) * \
    m.sin(dLon/2) * m.sin(dLon/2)

  c = 2 * m.atan2(m.sqrt(a), m.sqrt(1-a))
  d = Re * c

  return d * 1000.0 # meters

##
# \brief Calculate the geographic position at a small distance from the given
# position.
#
# The WGS84 spheroid model of the Earth is used. Only small delta distances
# are valid (<100km).
#
# \param lat    Starting geographic latitude (degrees)
# \param lon    Starting geographic longitude (degrees)
# \param dx     Delta position along N-S line (meters)
# \param dy     Delta position along W-E line (meters)
#
# \return New delta geographic latitude and longitude in degrees.
#
def geopos(lat, lon, dx, dy):
  rlat = radians(lat)
  rlon = radians(lon)

  m_per_deg_lat =   111132.92 \
                  - 559.82 * m.cos(2.0 * rlat) \
                  + 1.175 * m.cos(4.0 * rlat) \
                  - 0.0023 * m.cos(6.0 * rlat)

  m_per_deg_lon =   111412.84 * m.cos(rlat) \
                    - 93.5 * m.cos(3.0 * rlat) \
                    + 0.118 * m.cos(5.0 * rlat)

  dlat = dx/m_per_deg_lat
  dlon = dy/m_per_deg_lon

  return lat + dlat, lon + dlon

#def ascii_encode_dict(data):
#  ascii_encode = lambda x: x.encode('ascii')
#  return dict(map(ascii_encode, pair) for pair in data.items())

def ascii_encode_dict(data):
  ascii_encode = lambda x: x.encode('ascii') if isinstance(x, unicode) else x 
  return dict(map(ascii_encode, pair) for pair in data.items())

##
def goJson(jsonfilename):
  try:
    fp = open(jsonfilename)
  except IOError:
    print "Cannot open file {0}".format(jsonfilename)
  try:
    d = json.load(fp, object_hook=ascii_encode_dict)
  except ValueError, inst:
    print inst
    sys.exit(4)
  fp.close()
  print d

  k0 = 'starting_longitude'
  k1 = 'starting_latitude'
  if not d.has_key(k0):
    print "No {0} specified.".format(k0)
    sys.exit(2)
  elif SomeGeoCoords.has_key(d[k0]):
    Input[k1], Input[k0] = SomeGeoCoords[d[k0]]
  else:
    Input[k0] = float(d[k0])
    if not d.has_key(k1):
      print "No {0} specified.".format(k1)
      sys.exit(2)
    Input[k1] = float(d[k1])

  k = 'scale'
  if d.has_key(k):
    Input[k] = float(d[k])

  k = 'rotate'
  if d.has_key(k):
    Input[k] = float(d[k])

  k = 'translate'
  if d.has_key(k):
    Input[k] = (float(d[k][0]), float(d[k][1]))

  k = 'path'
  if not d.has_key(k):
    print "No {0} specified.".format(k)
    sys.exit(2)
  Input[k] = d[k]

  processPath()

##
def goInteractive():
  s = raw_input('Enter starting longitude (degrees) or\n' \
                '  known position (name) [Boulder]: ')

  # Starting lon,lat with default Boulder
  if len(s) == 0:
    s = 'Boulder'
    lat0, lon0 = SomeGeoCoords[s]
    print "Using {0} as the starting location.".format(s)
  elif SomeGeoCoords.has_key(s):
    lat0, lon0 = SomeGeoCoords[s]
    print "Using {0} as the starting location.".format(s)
  else:
    lon0 = float(s)
    s = raw_input('Enter starting latitude (degrees): ')
    lat0 = float(s)

  print "Starting longitude,latitude {0},{1}".format(lon0, lat0)

  Input['starting_longitude'] = lon0
  Input['starting_latitude']  = lat0

  scale = 1.0
  s = raw_input('Enter scale [1.0]: ')
  if len(s) > 0:
    scale = float(s)
  Input['scale'] = scale 

  translate = (0.0, 0.0)
  s = raw_input('Enter translate x y (meters) [0.0 0.0]: ')
  if len(s) > 0:
    args = s.split()
    translate = (float(args[0]), float(args[1]))
  Input['translate'] = translate

  rotate = 0.0
  s = raw_input('Enter rotate (degrees) [0.0]: ')
  if len(s) > 0:
    rotate = float(s)
  Input['rotate'] = rotate 

  print "2D affine transformation: scale {0}, rotate {1}, translate {2}".format(
      scale, rotate, translate)

  print "Enter delta x,y meters from starting latitude,longitude.\n" \
        "The x+ is due North, y+ is due West\n" \
        "  ('q' to terminate path)"

  while True:
    s = raw_input('x y (meters): ')
    args = s.split()
    if args[0] == 'q' or args[0] == 'Q':
      break
    x = float(args[0])
    y = float(args[1])

    Input['path'].append((x, y))

  processPath()

##
def printInput():
  print " ** Input:"
  print "Starting longitude: {0}".format(Input['starting_longitude'])
  print "Starting latitude:  {0}".format(Input['starting_latitude'])
  print "Scale:              {0}".format(Input['scale'])
  print "Rotate:             {0}".format(Input['rotate'])
  print "Translate:          {0}".format(Input['translate'])
  print "Input Path:"
  for pt in Input['path']:
    print "  {0}".format(pt)
  print

##
def transform(x, y, affine):
  augvec = np.matrix([[x], [y], [1.0]])
  t = affine * augvec
  return t.item(0, 0), t.item(1, 0)

##
def processPath():
  printInput()

  lon0      = Input['starting_longitude']
  lat0      = Input['starting_latitude']
  scale     = Input['scale']
  rotate    = radians(Input['rotate'])
  translate = Input['translate']
  geopath = []

  affine = np.matrix([
      [scale * m.cos(rotate), m.sin(rotate), translate[0]],
      [-m.sin(rotate), scale * m.cos(rotate), translate[1]],
      [0.0, 0.0, 1.0]])

  print "Transformed: "

  for x, y in Input['path']:
    x, y = transform(x, y, affine)
    print "  [{0}, {1}]".format(x, y)
    lat, lon = geopos(lat0, lon0, x, y)
    geopath.append([lon, lat])

  # close path
  geopath.append(geopath[0])

  print
  print "Output: "

  print geopath


if __name__ == '__main__':
  argv = sys.argv

  # parse command-line options
  try:
    opts, args = getopt.getopt(argv[1:], "?h", ['help', ''])
  except getopt.error, msg:
    print "Unknown option. See '%s --help'" % (__file__)
    sys.exit(0)

  for opt, optarg in opts:
    if opt in ('-h', '--help', '-?'):
      print """\
Usage: latlongmeters.py [jsonfile]
       latlongmeters.py --help

Description:
  If a Json file is specified, the file is parsed. Otherwise, interactive
  mode is entered to gather the data. The output is a list of geographic
  positions in longitude, latitude order. The output is compatible
  with Geofrenzy Feature Collections 'coordinates' field Json files.
  a Json file is specified, the file is parsed to output a list of

  List of known geographic positions:
  {0}

See Also:
  The shape files triangle.json, ..., tee.json serve as examples.
""".format(SomeGeoCoords.keys())
      sys.exit(0)

  if len(argv) > 1:
    goJson(argv[1])
  else:
    goInteractive()
