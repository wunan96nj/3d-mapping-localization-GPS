from collections import namedtuple
import uuid
from datetime import datetime
import base64
import requests
import json


def ConvertToBase64(src_filepath):
    with open(src_filepath, 'rb') as imageFileAsBinary:
        fileContent = imageFileAsBinary.read()
        b64_encoded_img = base64.b64encode(fileContent)
        return b64_encoded_img

def create_geopose_request(img_file_base64_string, lat, lon):


    """
    Create request body for geopose
    :param img_file_base64_string:
    :param lat:
    :param lon:
    :return:
    """
    
    data = {
        "id": str(uuid.uuid4()),
        "timestamp": str(datetime.now()),
        "type": "geopose",
        "sensors": [
            {
                "id": "0",
                "type": "camera"
            },
            {
                "id": "1",
                "type": "geolocation"
            }
        ],
        "sensorReadings": [
            {
                "timestamp": str(datetime.now()),
                "sensorId": "0",
                "reading": {
                    "sequenceNumber": 0,
                    "imageFormat": "JPG",
                    "size": [3072, 2304],
                    "imageOrientation": {
                        "mirrored": False,
                        "rotation": 0
                    },
                    "imageBytes": img_file_base64_string
                }
            },
            {
                "timestamp": str(datetime.now()),
                "sensorId": "1",
                "reading": {
                    "latitude": float(lat),
                    "longitude": float(lon),
                    "altitude": 0,
                    "accuracy": 0,
                    "altitudeAccuracy": 0,
                    "heading": 0,
                    "speed": 0,
                }
            }
        ]
    }
    json_data = json.dumps(data)
    return json_data
    

def main():
    api_url = "http://localhost:5666/capture-photo/getgeoposreq"
    image_name = "P1180141.jpg"
    uploadImagePath = "../client/images/" + image_name
    img_file_base64_string = str(ConvertToBase64(uploadImagePath), 'utf-8')
    json_data = create_geopose_request(img_file_base64_string, 1, 1)
    ret = requests.post(api_url, data=json_data)
    ret = ret.json()
    print(ret)

    json_data = json_data = json.dumps({"image_name": image_name})
    requests.post("http://localhost:5666/capture-photo/goepose_gt", data=json_data)


    return

if __name__ == '__main__':
    main()