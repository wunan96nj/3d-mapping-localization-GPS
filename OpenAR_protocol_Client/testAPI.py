#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from unicodedata import mirrored
import numpy as np
from numpy.linalg import inv
import shutil
import json
from flask import Flask, jsonify, request
from flask_restful import reqparse, Api, Resource
import os
import base64
import requests

from scipy.spatial.transform import Rotation as R
import math


app = Flask(__name__)
api = Api(app)

parser = reqparse.RequestParser()
parser.add_argument('task', type=str)

def ecef_to_geodetic(x, y, z):
   # Convert from ECEF cartesian coordinates to 
   # latitude, longitude and height.  WGS-84
    x2 = x ** 2 
    y2 = y ** 2 
    z2 = z ** 2 
 
    a = 6378137.0000    # earth radius in meters
    b = 6356752.3142    # earth semiminor in meters 
    pi = 3.14159265359
    e = math.sqrt (1-(b/a)**2) 
    b2 = b*b 
    e2 = e ** 2 
    ep = e*(a/b) 
    r = math.sqrt(x2+y2) 
    r2 = r*r 
    E2 = a ** 2 - b ** 2 
    F = 54*b2*z2 
    G = r2 + (1-e2)*z2 - e2*E2 
    c = (e2*e2*F*r2)/(G*G*G) 
    s = ( 1 + c + math.sqrt(c*c + 2*c) )**(1/3) 
    P = F / (3 * (s+1/s+1)**2 * G*G) 
    Q = math.sqrt(1+2*e2*e2*P) 
    ro = -(P*e2*r)/(1+Q) + math.sqrt((a*a/2)*(1+1/Q) - (P*(1-e2)*z2)/(Q*(1+Q)) - P*r2/2) 
    tmp = (r - e2*ro) ** 2 
    U = math.sqrt( tmp + z2 ) 
    V = math.sqrt( tmp + (1-e2)*z2 ) 
    zo = (b2*z)/(a*V) 
 
    height = U*( 1 - b2/(a*V) ) 
    
    lat = math.atan( (z + ep*ep*zo)/r ) 
 
    temp = math.atan(y/x) 
    if x >=0 :    
        long = temp 
    elif (x < 0) & (y >= 0):
        long = pi + temp 
    else :
        long = temp - pi 
 
    lat0 = lat/(pi/180) 
    lon0 = long/(pi/180) 
    h0 = height 
 
    return lat0, lon0, h0



def enu_to_ecef(xEast, yNorth, zUp, lat0, lon0, h0):
    a = 6378137
    b = 6356752.3142
    f = (a - b) / a
    e_sq = f * (2-f)
    pi = 3.14159265359
    lamb = math.radians(lat0)
    phi = math.radians(lon0)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)
 
    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
 
    x0 = (h0 + N) * cos_lambda * cos_phi
    y0 = (h0 + N) * cos_lambda * sin_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda
 
    t = cos_lambda * zUp - sin_lambda * yNorth
 
    zd = sin_lambda * zUp + cos_lambda * yNorth
    xd = cos_phi * t - sin_phi * xEast 
    yd = sin_phi * t + cos_phi * xEast
 
    x = xd + x0 
    y = yd + y0 
    z = zd + z0 
 
    return x, y, z


def enu_to_geodetic(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref):
 
    x,y,z = enu_to_ecef(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref)
 
    return ecef_to_geodetic(x,y,z)

def correct(q):
    ret = np.append(q[1:], q[0])
    return ret

def qt_to_posR(tvec, qvec):
    matrix_w2c = R.from_quat(correct(qvec)).as_matrix()
    t_w2c = tvec
    ret_pos = np.dot(-inv(matrix_w2c), t_w2c)
    ret_R = R.from_matrix(inv(matrix_w2c)).as_quat()

    return ret_pos, ret_R

'''class Sensor(object):
    def _init_(self, id, type):
        self.id = id
        self.type = type

class GetGeoPos(Resource):
    def post(self):
        json_data = request.get_json(force=True)
        longitude = json_data['logitude']
        latitude = json_data['latitude']
        elipsoidHeight = json_data['elipsoidHeight']
        quaternion = json_data['quaternion']
        quaternion_x = quaternion['x']
        quaternion_y = quaternion['y']
        quaternion_z = quaternion['z']
        quaternion_w = quaternion['w']
        print("longitude: " + longitude)
        print("latitude: " + latitude)
        print("elipsoidHeight: " + elipsoidHeight)
        print("quanternion_x: " + quaternion_x)
        print("quanternion_y: " + quaternion_y)
        print("quanternion_z: " + quaternion_z)
        print("quanternion_w: " + quaternion_w)
        return'''

@app.route('/capture-photo/getgeoposreq', methods=['GET', 'POST'])
def GetGeoPosReq():
    uploadimageb64 = None
    image_seqnum = None


    print("********************************************************************************")
    json_data = request.get_json(force=True)
    geo_id = json_data['id']
    timestamp = json_data['timestamp']
    req_type = json_data['type']
    sensors_trunk = json_data['sensors']
    sensorReadings_trunk = json_data['sensorReadings']
    print("id: " + geo_id)
    print("timestamp: " + timestamp)
    print("type: " + req_type)
    for i in range(len(sensors_trunk)):
        sensor_unit = sensors_trunk[i]
        print("sensor id: " + sensor_unit['id'])
        print("sensor type: " + sensor_unit['type'])
    for i in range(len(sensorReadings_trunk)):
        sensorReading_unit = sensorReadings_trunk[i]
        print("sensor reading timestamp: " + sensorReading_unit['timestamp'])
        print("sensor reading id: " + sensorReading_unit['sensorId'])
        #print("sensor reading privacy: " + sensorReading_unit['privacy'])
        reading = sensorReading_unit['reading']

        # Camera reading
        if (sensorReading_unit['sensorId'] == '0'):
            print("This is the Camera sensor reading!")
            sequence_number = reading['sequenceNumber']
            image_seqnum = sequence_number
            image_format = reading['imageFormat']
            camera_size = reading['size']
            imageBytes = reading['imageBytes']
            camera_reading_orientation = reading['imageOrientation']
            mirrored = camera_reading_orientation['mirrored']
            rotation = camera_reading_orientation['rotation']
            print("camera sequence number: ", sequence_number)
            print("image format: ", image_format)
            print("camera size(width, height): ", camera_size)
            write_to_file(imageBytes, "./test.jpg", True)
            uploadimageb64 = imageBytes
            #print("imageBytes: ", imageBytes)
            print("is image mirrored: ", mirrored)
            print("image rotation: ", rotation)

        # Geolocation reading
        if (sensorReading_unit['sensorId'] == '1'):
            print("This is the Geolocation sensor reading!")
            latitude = reading['latitude']
            longtitude = reading['longitude']
            altitude = reading['altitude']
            accuracy = reading['accuracy']
            altitudeAccuracy = reading['altitudeAccuracy']
            heading = reading['heading']
            speed = reading['speed']
            print("latitude: ", latitude)
            print("longtitude: ", longtitude)
            print("altitude: ", altitude)
            print("accuracy: ", accuracy)
            print("altitudeAccuracy: ", altitudeAccuracy)
            print("heading: ", heading)
            print("speed: ", speed)
        
        # Wifi reading
        if (sensorReading_unit['sensorId'] == '2'):
            print("This is the Wifi sensor reading!")
            bssid = reading['BSSID']
            frequency = reading['frequency']
            rssi = reading['RSSI']
            ssid = reading['SSID']
            scanTimeStart = reading['scanTimeStart']
            scanTimeEnd = reading['scanTimeEnd']
            print("bssid: ", bssid)
            print("frequency: ", frequency)
            print("rssi; ", rssi)
            print("ssid: ", ssid)
            print("scanTimeStart: ", scanTimeStart)
            print("scanTimeEnd: ", scanTimeEnd)
        
        # Bluetooth reading
        if (sensorReading_unit['sensorId'] == '3'):
            print("This is the Bluetooth sensor reading!")
            address = reading['address']
            rssi = reading['RSSI']
            name = reading['name']
            print("address: ", address)
            print("rssi: ", rssi)
            print("name: ", name)
        
        # Accelerometer reading and Gyroscope reading and Magnetometer reading
        if (sensorReading_unit['sensorId'] == '4' or sensorReading_unit['sensorId'] == '5' or sensorReading_unit['sensorId'] == '6'):
            if (sensorReading_unit['sensorId'] == '4'):
                print("This is the Accelerometer sensor reading!")
            if (sensorReading_unit['sensorId'] == '5'):
                print("This is the Gyroscope sensor reading!")
            if (sensorReading_unit['sensorId'] == '6'):
                print("This is the Magnetometer sensor reading!")
            x = reading['x']
            y = reading['y']
            z = reading['z']
            print("x: ", x)
            print("y: ", y)
            print("z: ", z)

    #Send to mapping api for localization

    username = 'sample_user'
    password = 'pass'
    bank = 0
    mappingapi_url = "http://localhost:5444/capture-photo"
    token = "192b47014ee982495df0a08674ac49a11eca4cb4427e3115a0254b89d07587cc"
    (ret_image_name, ret_qvec, ret_tvec, lat_ref, lon_ref, h_ref) = QueryLocal(
        mappingapi_url, token, uploadimageb64, sequence_number, bank, username, password)
    
    print("Print out localization results (name, q, t, GPS)")
    print(ret_image_name, ret_qvec, ret_tvec, lat_ref, lon_ref, h_ref)


    #ret = {"ret_image_name": ret_image_name, "ret_qvec": ret_qvec, "ret_tvec": ret_tvec}

    enu_pos, orient = qt_to_posR(ret_tvec, ret_qvec)


    wgs84_pos = enu_to_geodetic(enu_pos[0], enu_pos[1], enu_pos[2], lat_ref, lon_ref, h_ref)
    ret = {
        "id": geo_id, 
        "timestamp": timestamp, 
        "accuracy": 0,
        "type": req_type,
        "geopose": {
            "position":{
                "lon": wgs84_pos[0],
                "lat": wgs84_pos[1],
                "h": wgs84_pos[2]
            },
            "quaternion":{
                "x": orient[0],
                "y": orient[1],
                "z": orient[2],
                "w": orient[3]
            }
        }

    }

    return jsonify(json.dumps(ret))


def write_to_file(content_s, file_full_path, is_base64):
    if is_base64:
        base64_bytes = content_s.encode('ascii')
        file_bytes = base64.b64decode(base64_bytes)
    else:
        file_bytes = content_s.encode('ascii')
    with open(file_full_path, 'wb') as f:
        f.write(file_bytes)
    return

def QueryLocal(url, token, uploadimageb64, sequence_number, bank, username, password):
    complete_url = url + '/querylocal'
    #(image_dir, image_name) = os.path.split(uploadImagePath)
    #image_name = image_name.split('.')[0] + ".jpg"
    data = {
        "token": token,
        "bank": bank,
        "b64": uploadimageb64,
        "image_name": str(sequence_number)
    }
    json_data = json.dumps(data)
    return_obj = json.loads(requests.post(complete_url, data=json_data, auth=(username, password)).json())
    #print(return_obj)
    ret_image_name = return_obj[0]
    ret_qvec = return_obj[1]
    ret_tvec = return_obj[2]
    lat_ref, lon_ref, h_ref = return_obj[3], return_obj[4], return_obj[5]
    return (ret_image_name, ret_qvec, ret_tvec, lat_ref, lon_ref, h_ref)



@app.route('/capture-photo/goepose_gt', methods=['GET', 'POST'])
def imageBinInfo(url, token, image_name, bank, username, password):
    # image_name = image_name.split('.')[0]
    print("ImageBinInfo...bank: " + str(bank))
    print("ImageBinInfo...image_name: " + str(image_name))
    complete_url = url + '/imagebininfo'
    data = {
        "token": token,
        "bank": bank,
        "image_name": image_name
    }
    json_data = json.dumps(data)
    return_obj = json.loads(requests.post(complete_url, data=json_data, auth=(username, password)).json())
    return return_obj     
        

## Actually setup the Api resource routing here
##
# api.add_resource(TodoList, '/todos')
# api.add_resource(Todo, '/todos/<todo_id>')
# http://localhost:5666/

#api.add_resource(GetGeoPos,'/getgeopos')

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5666, debug=True)
