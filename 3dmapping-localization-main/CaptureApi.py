#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import sys
import os

#print(sys.path)
import numpy
import shutil
import json
from flask import Flask, jsonify, request
from flask_restful import reqparse, Api, Resource
import os
from map3d.util import QueryLocalUtil, Utils, MyEnv
from map3d.util.calc import read_model
from environs import Env
from flask_httpauth import HTTPBasicAuth

app = Flask(__name__)
api = Api(app)
auth = HTTPBasicAuth()

env = Env()
env.read_env(path='server.env')
COLMAP = env.str('COLMAP')
root_dir = env.str('root_dir')
print(root_dir)
image_bin_name = env.str('image_bin_name')
database_name = env.str('database_name')


@auth.verify_password
def verify_password(username, password):
    if username == 'sample_user' and password == 'pass':
        return True
    return False


def init():
    return


@app.route('/capture-photo/captureb64', methods=['GET', 'POST'])
@auth.login_required
def CapturePhoto():
    # file_uuid = uuid.uuid4().hex;
    json_data = request.get_json(force=True)
    token = json_data['token']
    bank = json_data['bank']
    run = json_data['run']
    index = json_data['index']
    anchor = json_data['anchor']

    #Position Here, not translation
    px = json_data['px']
    py = json_data['py']
    pz = json_data['pz']
    r00 = json_data['r00']
    r01 = json_data['r01']
    r02 = json_data['r02']
    r10 = json_data['r10']
    r11 = json_data['r11']
    r12 = json_data['r12']
    r20 = json_data['r20']
    r21 = json_data['r21']
    r22 = json_data['r22']
    fx = json_data['fx']
    fy = json_data['fy']
    ox = json_data['ox']
    oy = json_data['oy']
    image_name = json_data['image_name']
    image_name = image_name.split('.')[0]
    b64 = json_data['b64']
    username = auth.username();
    (workspace_dir, image_base_dir, json_base_dir, sparse_dir, database_dir, col_bin_dir) = MyEnv.get_env_total_dir(
        username,
        root_dir, bank)
    (jpg_file_full_path, json_file_path) = MyEnv.get_jpg_json_file_path(image_base_dir, json_base_dir, image_name)

    CapturePhoto_save_files(json_data, jpg_file_full_path, json_file_path)

    return jsonify(image_name=image_name,
                   jpg_file_full_path=jpg_file_full_path,
                   json_file_path=json_file_path)


def CapturePhoto_save_files(json_data, image_file_full_path, json_file_full_path):
    print(image_file_full_path)
    b64 = json_data['b64']
    json_data['b64'] = "omitted"
    Utils.write_to_file(b64, image_file_full_path, True)
    Utils.write_to_file(json_data, json_file_full_path, False)
    return


# construction by images
@app.route('/capture-photo/construct', methods=['GET', 'POST'])
@auth.login_required
def StartMapConstruction():
    print("StartMapConstruction BEGIN")
    json_data = request.get_json(force=True)
    bank = json_data['bank']
    feature_dim = json_data['feature_dim']
    StartMapConstruction_build(feature_dim, bank)
    # Utils.gen_newdb(sparse_dir, database_name, feature_dim, bank )
    # Utils.remove_build_useless_files(sparse_dir, feature_dim, bank )
    print("StartMapConstruction FIN")
    return jsonify();


def StartMapConstruction_build(feature_dim, bank):
    print("StartMapConstruction build() start.....")
    username = auth.username();
    (workspace_dir, image_base_dir, json_base_dir, sparse_dir, database_dir, col_bin_dir) = MyEnv.get_env_total_dir(
        username, root_dir, bank)
    print("1. feature_extractor")
    Utils.feature_colmap(COLMAP, database_name, database_dir, image_base_dir)
    # Utils.feature_cv(tmp_database_dir + database_name, image_dir)
    print("2. Matching")
    Utils.match_colmap(COLMAP, database_name, database_dir, image_base_dir)

    print("3. point_triangulator")
    #Utils.point_triangulator_colmap(COLMAP, database_name, sparse_dir, database_dir, image_base_dir)
    print("StartMapConstruction build() end .....")

    print("4. model Aligner")
    print(database_dir)
    Utils.model_aligner_colmap(COLMAP, sparse_dir, database_dir, json_base_dir)

    print("StartMapConstruction build() end .....")
    return


@app.route('/capture-photo/clear', methods=['GET', 'POST'])
@auth.login_required
def ClearWorkspace():
    print("ClearWorkspace BEGIN, ")
    json_data = request.get_json(force=True)
    bank = json_data['bank']
    username = auth.username();
    (workspace_dir, image_base_dir, json_base_dir, sparse_dir, database_dir, col_bin_dir) = MyEnv.get_env_total_dir(
        username, root_dir,
        bank)
    if os.path.exists(workspace_dir):
        shutil.rmtree(workspace_dir, ignore_errors=True)
    '''image_dir = image_base_dir + str(bank) + "/"
    json_dir = json_base_dir + str(bank) + "/"
    sparse_dir_bank = sparse_dir + str(bank) + "/"
    if os.path.exists(image_dir):
        shutil.rmtree(image_dir, ignore_errors=True)
    if os.path.exists(json_dir):
        shutil.rmtree(json_dir, ignore_errors=True)
    if os.path.exists(sparse_dir_bank):
        shutil.rmtree(sparse_dir_bank, ignore_errors=True)'''
    return jsonify();
    print("ClearWorkspace FIN")


@app.route('/capture-photo/querylocal', methods=['GET', 'POST'])
@auth.login_required
def QueryLocal():
    print("QueryLocal BEGIN, ")
    json_data = request.get_json(force=True)
    bank = json_data['bank']
    b64 = json_data['b64']
    image_name = json_data['image_name']
    username = auth.username();
    (workspace_dir, image_base_dir, json_base_dir, sparse_dir, database_dir, col_bin_dir) = MyEnv.get_env_total_dir(
        username, root_dir, bank)
    (
        image_name_prefix, upload_image_file_full_path,
        upload_database_file_full_path,
        upload_image_tmp_dir) = MyEnv.establish_env(image_name,
                                                    sparse_dir)
    print("QueryLocal image_name_prefix: " + image_name_prefix)
    print(
        "QueryLocal upload_image_file_full_path: " + upload_image_file_full_path)
    print(
        "QueryLocal upload_database_file_full_path: " + upload_database_file_full_path)

    QueryLocalUtil.save_image(b64, bank, upload_image_tmp_dir,
                              upload_image_file_full_path)
    QueryLocalUtil.get_feature_upload(COLMAP, image_name_prefix + ".db",
                                      upload_image_tmp_dir)
    (image_name_jpg, q, t) = QueryLocalUtil.compare_upload_base_local(
        sparse_dir,
        col_bin_dir,
        upload_database_file_full_path,
        image_name_prefix + ".jpg")
    print("QueryLocal (image_name_jpg, q, t):" + str(
        (image_name_jpg, q, t)) + " FIN")
    return jsonify(json.dumps((image_name_jpg, q, t), cls=Utils.NDArrayEncoder))

    ##


@app.route('/capture-photo/cvquerylocal', methods=['GET', 'POST'])
@auth.login_required
def CVQueryLocal():
    print("CVQueryLocal BEGIN, ")
    json_data = request.get_json(force=True)
    bank = json_data['bank']
    fg_des = json_data['fg_des']
    fg_kp = json_data['fg_kp']
    params = json_data['params']
    image_name_jpg = json_data['image_name']

    fg_kp = numpy.array(fg_kp)
    fg_des = numpy.array(fg_des).astype(numpy.uint8)
    params = numpy.array(params)
    #
    username = auth.username();
    (workspace_dir, image_base_dir, json_base_dir, sparse_dir, database_dir, col_bin_dir) = MyEnv.get_env_total_dir(
        username, root_dir, bank)

    print("CVQueryLocal image_name_jpg: " + image_name_jpg)
    print("CVQueryLocal sparse_dir_bank: " + sparse_dir)
    (image_name_jpg, q, t) = QueryLocalUtil.compare_upload_base_local_cv(sparse_dir, col_bin_dir, image_name_jpg,
                                                                         fg_kp,
                                                                         fg_des, params)
    print("CVQueryLocal (image_name_jpg, q, t):" + str(
        (image_name_jpg, q, t)) + " FIN")
    return jsonify(json.dumps((image_name_jpg, q, t), cls=Utils.NDArrayEncoder))

    ##


@app.route('/capture-photo/imagebininfo', methods=['GET', 'POST'])
@auth.login_required
def ImageBinInfo():
    print("ImageBinInfo BEGIN, ")
    json_data = request.get_json(force=True)
    bank = json_data['bank']
    the_image_name = json_data['image_name']
    username = auth.username();
    (workspace_dir, image_base_dir, json_base_dir, sparse_dir, database_dir, col_bin_dir) = MyEnv.get_env_total_dir(
        username, root_dir, bank)
    image_bin_path = MyEnv.image_bin_path(sparse_dir, image_bin_name)
    (image_id, qvec, tvec,
     camera_id, image_name,
     xys, point3D_ids) = read_model.read_images_binary_for_one(image_bin_path,
                                                               the_image_name)
    print("ImageBinInfo (image_id, qvec, tvec, camera_id, image_name, xys, point3D_ids):" + str(
        (image_id, qvec, tvec, camera_id, image_name, xys, point3D_ids)) + " FIN")
    return jsonify(
        json.dumps((image_id, qvec, tvec, camera_id, image_name, xys, point3D_ids), cls=Utils.NDArrayEncoder))


@app.route('/capture-photo/query3dcloudpoint', methods=['GET', 'POST'])
@auth.login_required
def Query3DCouldPoint():
    print("Query3DCouldPoint BEGIN, ")
    json_data = request.get_json(force=True)
    bank = json_data['bank']
    params = json_data['params']
    #
    username = auth.username();
    (workspace_dir, image_base_dir, json_base_dir, sparse_dir, database_dir, col_bin_dir) = MyEnv.get_env_total_dir(
        username, root_dir, bank)
    print("Query3DCouldPoint sparse_dir: " + sparse_dir)
    (db_points_pos, db_points_des, dp_points_rgb) = Utils.load_all_3dmap_cloud_point(sparse_dir, col_bin_dir)
    print("Query3DCouldPoint (db_points_pos, db_points_des, dp_points_rgb):" + str(
        (db_points_pos, db_points_des, dp_points_rgb)) + " FIN")
    return jsonify(json.dumps((db_points_pos, db_points_des, dp_points_rgb), cls=Utils.NDArrayEncoder))


## Actually setup the Api resource routing here
##
# api.add_resource(TodoList, '/todos')
# api.add_resource(Todo, '/todos/<todo_id>')
# http://localhost:5444/capture-photo
# api.add_resource(CapturePhoto, '/capture-photo/captureb64')
# api.add_resource(StartMapConstruction, '/capture-photo/contruct')
# api.add_resource(ClearWorkspace, '/capture-photo/clear')
# api.add_resource(QueryLocal, '/capture-photo/querylocal')
# api.add_resource(CVQueryLocal, '/capture-photo/cvquerylocal')
# api.add_resource(ImageBinInfo, '/capture-photo/imagebininfo')
# api.add_resource(Query3DCouldPoint, '/capture-photo/query3dcloudpoint')

if __name__ == '__main__':
    init();
    app.run(host='0.0.0.0', port=5444, debug=True)
