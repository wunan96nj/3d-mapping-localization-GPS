#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import numpy
import cv2
import base64
import shutil
from json import JSONEncoder
import os
import subprocess
from map3d.util.db import database, write_to_nw_db
from map3d.util.calc import get_point_pos_des, get_point_feature
import open3d
import json

import math
 
a = 6378137
b = 6356752.3142
f = (a - b) / a
e_sq = f * (2-f)
pi = 3.14159265359
 
def geodetic_to_ecef(lat, lon, h):
    # (lat, lon) in degrees
    # h in meters
    lamb = math.radians(lat)
    phi = math.radians(lon)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)
 
    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)
 
    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda
 
    return x, y, z
 
def ecef_to_enu(x, y, z, lat0, lon0, h0):
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
 
    xd = x - x0
    yd = y - y0
    zd = z - z0
 
    t = -cos_phi * xd -  sin_phi * yd
 
    xEast = -sin_phi * xd + cos_phi * yd
    yNorth = t * sin_lambda  + cos_lambda * zd
    zUp = cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd + sin_lambda * zd
 
    return xEast, yNorth, zUp
 
def enu_to_ecef(xEast, yNorth, zUp, lat0, lon0, h0):
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
 
def ecef_to_geodetic(x, y, z):
   # Convert from ECEF cartesian coordinates to 
   # latitude, longitude and height.  WGS-84
    x2 = x ** 2 
    y2 = y ** 2 
    z2 = z ** 2 
 
    a = 6378137.0000    # earth radius in meters
    b = 6356752.3142    # earth semiminor in meters 
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
 
 
def geodetic_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref):
 
    x, y, z = geodetic_to_ecef(lat, lon, h)
    
    return ecef_to_enu(x, y, z, lat_ref, lon_ref, h_ref)
 
def enu_to_geodetic(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref):
 
    x,y,z = enu_to_ecef(xEast, yNorth, zUp, lat_ref, lon_ref, h_ref)
 
    return ecef_to_geodetic(x,y,z)

class NDArrayEncoder(JSONEncoder):
    def default(self, obj):
        if isinstance(obj, numpy.ndarray):
            return obj.tolist()
        return JSONEncoder.default(self, obj)


# dp_points_rgb [0-255]
def write_xyz_to_point_cloud_file(db_points_pos, db_points_des, dp_points_rgb, ply_file_path):
    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(db_points_pos)
    dp_points_rgb = numpy.array(dp_points_rgb)
    pcd.colors = open3d.utility.Vector3dVector(dp_points_rgb.astype(numpy.float64) / 255.0)
    open3d.io.write_point_cloud(ply_file_path, pcd)
    return


def load_all_3dmap_cloud_point(sparse_dir, col_bin_dir):
    db_cameras, db_images, db_points = get_point_feature.read_cip(col_bin_dir)
    db_images_table, db_kp_table, db_des_table = get_point_feature.read_database(sparse_dir)
    db_points_pos, db_points_des, dp_points_rgb = get_point_feature.get_points_pos_des(db_cameras, db_images, db_points,
                                                                                       db_kp_table, db_des_table)
    return (db_points_pos, db_points_des, dp_points_rgb)


def correct_colmap_q(qvec):
    ret = numpy.roll(qvec, 1)
    return ret


def write_to_file(content_s, file_full_path, is_base64):
    if is_base64:
        base64_bytes = content_s.encode('ascii')
        file_bytes = base64.b64decode(base64_bytes)
        with open(file_full_path, 'wb') as f:
            f.write(file_bytes)
    else:
        #file_bytes = content_s.encode('ascii')
        with open(file_full_path, 'w') as fp:
            json.dump(content_s, fp)
    return


def feature_cv(database_path, img_folder, ):
    img_names = os.listdir(img_folder)
    print(img_names)
    if os.path.exists(database_path):
        os.remove(database_path)
    db = database.COLMAPDatabase.connect(database_path)
    db.create_tables()
    for i in range(len(img_names)):
        img_name = img_names[i]

        print("img_name:%s" % img_name)
        (model, width, height, params) = get_camera_info_cv()
        camera_id = db.add_camera(model, width, height, params)
        image_id = db.add_image(img_name, camera_id)
        (fg_kp, fg_des) = feature_one_image_cv(img_name, img_folder, model,
                                               width, height, params)
        print(fg_kp.shape)
        print(fg_des.shape)
        db.add_keypoints(image_id, fg_kp)
        db.add_descriptors(image_id, fg_des)
        db.commit()
    db.close()


def get_camera_info_cv():
    model, width, height, params = 0, 3072, 2304, numpy.array(
        (2457.6, 1536., 1152.))
    return (model, width, height, params)


def feature_one_image_cv(img_name, img_folder):
    img_path = img_folder + "/" + img_name
    img = cv2.imread(img_path, 0)
    sift = cv2.SIFT_create(10000)
    fg_kp, fg_des = sift.detectAndCompute(img, None)
    fg_kp = numpy.array([fg_kp[i].pt for i in range(len(fg_kp))])
    fg_des = numpy.array(fg_des).astype(numpy.uint8)
    return (fg_kp, fg_des)


def feature_colmap(COLMAP, database_name, tmp_database_dir, image_dir):
    '''pIntrisics = subprocess.Popen(
        [COLMAP, "feature_extractor", "--database_path",
         tmp_database_dir + database_name, "--image_path", image_dir,
         "--ImageReader.camera_model", "SIMPLE_PINHOLE", "--SiftExtraction.use_gpu", "false"])'''
    pIntrisics = subprocess.Popen(
        [COLMAP, "feature_extractor", "--database_path",
         tmp_database_dir + database_name, "--image_path", image_dir,
         "--ImageReader.camera_model", "SIMPLE_PINHOLE"])
    pIntrisics.wait()


def match_colmap(COLMAP, database_name, tmp_database_dir, image_dir ):
    pIntrisics = subprocess.Popen(
        [COLMAP, "exhaustive_matcher", "--database_path",
         tmp_database_dir + database_name, "--SiftMatching.use_gpu", "false"])
    '''pIntrisics = subprocess.Popen(
        [COLMAP, "exhaustive_matcher", "--database_path",
         tmp_database_dir + database_name])'''
    pIntrisics.wait()



def point_triangulator_colmap(COLMAP, database_name, sparse_dir,
                              tmp_database_dir, image_dir ):
    pIntrisics = subprocess.Popen(
        [COLMAP, "mapper", "--database_path",
         tmp_database_dir + database_name,
         "--image_path", image_dir, "--output_path",
         sparse_dir, "--Mapper.ba_refine_focal_length", "0",
         "--Mapper.ba_refine_extra_params", "0"])
    pIntrisics.wait()

def save_prior_position(json_base_dir, save_path):
    print(json_base_dir)
    file_list = os.listdir(json_base_dir)


    f  = open(save_path+ "coords.txt", 'w')
    lat_ref, lon_ref, h_ref = 0.0, 0.0, 0.0
    for fileindex in range(len(file_list)):
        with open(json_base_dir + file_list[fileindex]) as json_file:
            data = json.load(json_file)
            lat = data["px"]
            lon = data["py"]
            h = data["pz"]
            if fileindex == 0:
                lat_ref, lon_ref, h_ref = lat, lon, h

                with open(save_path + "map_origin.txt", 'w') as map_origin_f:
                    map_origin_f.write("%f %f %f\n" % (lat_ref, lon_ref, h_ref))

            xEast, yNorth, zUp = geodetic_to_enu(lat, lon, h, lat_ref, lon_ref, h_ref)

        f.write("%s %f %f %f\n" %(data["image_name"], xEast, yNorth, zUp))
    f.close()



def model_aligner_colmap(COLMAP, sparse_dir, tmp_database_dir, json_base_dir):
    save_prior_position(json_base_dir, tmp_database_dir)
    pIntrisics = subprocess.Popen(
        [COLMAP, "model_aligner", "--input_path", sparse_dir+"0/", "--output_path", sparse_dir+"0/", "--ref_is_gps", "0",
           "--ref_images_path", tmp_database_dir + "coords.txt", "--robust_alignment_max_error", "1", "--alignment_type", "custom"])
    pIntrisics.wait()


# def printImageBinInfo(uploadImagePath, image_bin_path="/Users/akui/Desktop/sparse/0/images.bin"):
#     (image_dir, the_image_name) = os.path.split(uploadImagePath)
#
#     return (image_id, qvec, tvec,
#             camera_id, image_name,
#             xys, point3D_ids)


'''
def gen_newdb(sparse_dir, database_name, feature_dim, bank ):
    print("StartMapConstruction gen_newdb() start .....")
    sparse_dir_bank = sparse_dir + str(bank) + "/"
    tmp_database_dir = sparse_dir_bank + "/temp/"
    print("sparse_dir_bank: " + sparse_dir_bank)
    print("tmp_database_dir: " + tmp_database_dir)
    print("1. write_to_nw_db.read_cip")
    cameras, images, points = write_to_nw_db.read_cip(sparse_dir_bank)
    print(cameras)
    print("2. write_to_nw_db.read_database")
    db_images, kp_table, des_table = write_to_nw_db.read_database(
        tmp_database_dir, feature_dim)

    print("3. write_to_nw_db.get_points_pos_des")
    points_pos, points_des, points_rgb = write_to_nw_db.get_points_pos_des(
        cameras, images,
        points,
        kp_table,
        des_table)

    # print(len(points))
    # print(len(points_pos))
    # print(len(points_des))
    # print(points)
    print(list(points_pos[-1]))
    print(list(points_des[-1]))
    print(list(points_rgb[-1]))
    print("4. write_to_nw_db.write_points3D_nw_db")
    write_to_nw_db.write_points3D_nw_db(points_pos, points_rgb, points_des,
                                        sparse_dir_bank + database_name)
    print("StartMapConstruction gen_newdb() end .....")
    return
'''
'''
def remove_build_useless_files(sparse_dir, feature_dim, bank ):
    print("StartMapConstruction remove_useless_files() start .....")
    sparse_dir_bank = sparse_dir + str(bank) + "/"
    tmp_database_dir = sparse_dir_bank + "temp/"
    if os.path.exists(tmp_database_dir):
        shutil.rmtree(tmp_database_dir, ignore_errors=True)
    if os.path.exists(sparse_dir_bank + "project.ini"):
        os.remove(sparse_dir_bank + "project.ini")
    if os.path.exists(sparse_dir_bank + "points3D.bin"):
        os.remove(sparse_dir_bank + "points3D.bin")

    print("StartMapConstruction remove_useless_files() end .....")
    return
'''


def main():
    base_bank_dir = "/Users/akui/Desktop/sparse/0"
    ply_file_path = "/Users/akui/Desktop/test.ply"
    (db_points_pos, db_points_des, dp_points_rgb) = load_all_3dmap_cloud_point(base_bank_dir)
    write_xyz_to_point_cloud_file(db_points_pos, db_points_des, dp_points_rgb, ply_file_path)
    return


if __name__ == "__main__":
    main()
