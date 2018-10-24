#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pos2vmd.py - convert joint position data to VMD

from __future__ import print_function

def usage(prog):
    print('usage: ' + prog + ' POSITION_FILE VMD_FILE')
    sys.exit()

import os
import re
from PyQt5.QtGui import QQuaternion, QVector4D, QVector3D, QMatrix4x4
from VmdWriter import VmdBoneFrame, VmdInfoIk, VmdShowIkFrame, VmdWriter
import argparse
import logging
import datetime
import numpy as np
import csv
from decimal import Decimal
from matplotlib import pyplot as plt
from collections import Counter
import math
import copy
from operator import itemgetter
              
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

level = {0:logging.ERROR,
            1:logging.WARNING,
            2:logging.INFO,
            3:logging.DEBUG}
verbose = 2

# ディクショナリ型で各ボーンごとのキーフレームリストを作成する
bone_frame_dic = {
    "上半身":[],
    "上半身2":[],
    "下半身":[],
    "首":[],
    "頭":[],
    "左肩":[],
    "左腕":[],
    "左ひじ":[],
    "右肩":[],
    "右腕":[],
    "右ひじ":[],
    "左足":[],
    "左ひざ":[],
    "右足":[],
    "右ひざ":[],
    "センター":[],
    "グルーブ":[],
    "左足ＩＫ":[],
    "右足ＩＫ":[]
}

# 関節位置情報のリストからVMDを生成します
def position_list_to_vmd_multi(positions_multi, positions_gan_multi, upright_file, vmd_file, smoothed_file, bone_csv_file, depth_file, start_frame_file, center_xy_scale, center_z_scale, xangle, mdecimation, idecimation, ddecimation, alignment, is_ik, heelpos, smooth_times, upright_target):
    writer = VmdWriter()
    
    # 開始フレームインデックス
    start_frame = load_start_frame(start_frame_file)
    logger.info("開始フレームインデックス: %d", start_frame)
    
    # 関節二次元情報を読み込み
    smoothed_2d = load_smoothed_2d(smoothed_file)

    # 上半身2があるかチェック
    is_upper2_body = is_upper2_body_bone(bone_csv_file)

    logger.info("角度計算開始")

    # 各関節角度の算出
    for frame, positions in enumerate(positions_multi):
        positions_gan = None
        if positions_gan_multi is not None:
            positions_gan = positions_gan_multi[frame]

        positions_to_frames(positions, positions_gan, smoothed_2d, frame, xangle, is_upper2_body)    

    logger.info("直立フレーム推定開始")

    # 体幹的に最も直立しているINDEX抽出
    upright_idxs = calc_upright_body()

    logger.info(upright_idxs)

    logger.info("センター計算開始")

    # センター調整
    target_upright_idx, target_start_pos = load_upright_target(upright_target)

    # センターの計算
    calc_center(smoothed_2d, bone_csv_file, upright_idxs, center_xy_scale, center_z_scale, heelpos, target_upright_idx, target_start_pos)

    depths = load_depth(depth_file)

    depth_all_frames = None
    if depths is not None:
        # 深度ファイルがある場合のみ、Z軸計算
        logger.info("センターZ計算開始")

        # センターZの計算
        depth_all_frames = calc_center_z(smoothed_2d, depths, start_frame, upright_idxs, center_xy_scale, center_z_scale, target_upright_idx, target_start_pos)

    # 角度をなめらかに
    smooth_angle(smooth_times, ["上半身", "上半身2", "下半身", "首", "頭", "左肩", "左腕", "左ひじ", "右肩",  "右腕", "右ひじ", "左足", "左ひざ", "右足", "右ひざ"])

    logger.info("IK計算開始")

    if is_ik:
        # IKの計算
        calc_IK(bone_csv_file, smoothed_2d, depth_all_frames, upright_idxs, heelpos)
    else:
        #　IKでない場合は登録除去
        bone_frame_dic["左足ＩＫ"] = []
        bone_frame_dic["右足ＩＫ"] = []
    
    # IKをなめらかに
    smooth_IK(smooth_times)

    # bf_x = []
    # bf_y = []
    # bf_z = []
    # for bf in bone_frame_dic["センター"][500:600]:
    #     bf_x.append(bf.position.x())
    #     bf_y.append(bf.position.y())
    #     bf_z.append(bf.position.z())

    # logger.info("bf_x")
    # logger.info(bf_x)
    # logger.info("bf_y")
    # logger.info(bf_y)
    # logger.info("bf_z")
    # logger.info(bf_z)

    # センターを滑らかに
    smooth_move(smooth_times, ["センター"])

    # 直立関連ファイルに情報出力
    # 直立IDX
    upright_file.write(str(upright_idxs[0]))
    upright_file.write("\n")
    # 先頭フレームのセンターpos
    center_pos = bone_frame_dic["センター"][0].position
    upright_file.write("center,{0},{1},{2}".format(center_pos.x(), center_pos.y(), center_pos.z()))
    upright_file.write("\n")
    # 先頭フレームの2D
    # logger.info("upright: %s", upright_idxs[0])
    for key in ["Neck", "RHip", "LHip", "RKnee", "LKnee", "RAnkle", "LAnkle"]:
        # logger.info("key: %s, v: %s", k, v)
        s2d = smoothed_2d[0][SMOOTHED_2D_INDEX[key]]
        # logger.info(s2d)
        upright_file.write("{0},{1},{2},{3}".format(key, s2d.x(), s2d.y(), s2d.z()))
        upright_file.write("\n")

    upright_file.close()

    logger.info("グルーブ移管開始")

    # グルーブ移管
    is_groove = set_groove(bone_csv_file)

    if mdecimation > 0 or idecimation > 0 or ddecimation > 0:
        
        base_dir = os.path.dirname(vmd_file)

        if alignment == True:
            logger.info("揃えて間引き開始")
            # 揃えて間引き
            decimate_bone_center_frames_array(base_dir, is_groove, mdecimation)
            
            if is_ik:                
                decimate_bone_ik_frames_array(base_dir, ["左足ＩＫ", "左足"], idecimation, ddecimation)
                decimate_bone_ik_frames_array(base_dir, ["右足ＩＫ", "右足"], idecimation, ddecimation)
            else:
                decimate_bone_ik_frames_array(base_dir, ["左足", "左ひざ"], idecimation, ddecimation)
                decimate_bone_ik_frames_array(base_dir, ["右足", "右ひざ"], idecimation, ddecimation)
                
            # decimate_bone_rotation_frames_array(["上半身"], ddecimation)
            # decimate_bone_rotation_frames_array(["下半身"], ddecimation)
            if is_upper2_body:
                decimate_bone_rotation_frames_array(["上半身", "上半身2", "下半身"], ddecimation)
            else:
                decimate_bone_rotation_frames_array(["上半身", "下半身"], ddecimation)

            decimate_bone_rotation_frames_array(["首", "頭"], ddecimation)
            decimate_bone_rotation_frames_array(["左ひじ", "左腕", "左肩"], ddecimation)
            decimate_bone_rotation_frames_array(["右ひじ", "右腕", "右肩"], ddecimation)
        else:
            logger.info("通常間引き開始")
            decimate_bone_center_frames_array(base_dir, is_groove, mdecimation)

            if is_ik:
                decimate_bone_ik_frames_array(base_dir, ["左足ＩＫ"], idecimation, ddecimation)
                decimate_bone_ik_frames_array(base_dir, ["右足ＩＫ"], idecimation, ddecimation)
            else:
                decimate_bone_rotation_frames_array(["左ひざ"], ddecimation)
                decimate_bone_rotation_frames_array(["右ひざ"], ddecimation)
                
            if is_upper2_body:
                decimate_bone_rotation_frames_array(["上半身2"], ddecimation)

            decimate_bone_rotation_frames_array(["上半身"], ddecimation)
            decimate_bone_rotation_frames_array(["下半身"], ddecimation)
            decimate_bone_rotation_frames_array(["左足"], ddecimation)
            decimate_bone_rotation_frames_array(["右足"], ddecimation)
            decimate_bone_rotation_frames_array(["首"], ddecimation)
            decimate_bone_rotation_frames_array(["頭"], ddecimation)
            decimate_bone_rotation_frames_array(["左ひじ"], ddecimation)
            decimate_bone_rotation_frames_array(["左腕"], ddecimation)
            decimate_bone_rotation_frames_array(["左肩"], ddecimation)
            decimate_bone_rotation_frames_array(["右ひじ"], ddecimation)
            decimate_bone_rotation_frames_array(["右腕"], ddecimation)
            decimate_bone_rotation_frames_array(["右肩"], ddecimation)

    logger.info("VMD出力開始")

    # logger.info("upper result: f={0}, x={1}, y={2}, z={3}".format(701, bone_frame_dic["上半身"][701].rotation.toEulerAngles().x(), bone_frame_dic["上半身"][701].rotation.toEulerAngles().y(), bone_frame_dic["上半身"][701].rotation.toEulerAngles().z()))

    # ディクショナリ型の疑似二次元配列から、一次元配列に変換
    bone_frames = []
    for k,v in bone_frame_dic.items():
        for bf in v:
            bone_frames.append(bf)

    # vmd出力ファイルにフレーム番号再設定
    vmd_file = vmd_file.replace("[uDDDD]", "u{0:05d}".format(upright_idxs[0]))

    # writer.write_vmd_file(vmd_file, bone_frames, showik_frames, expression_frames)
    showik_frames = make_showik_frames(is_ik)
    writer.write_vmd_file(vmd_file, bone_frames, showik_frames)

    logger.info("VMDファイル出力完了: {0}".format(vmd_file))


# 3D推定位置から関節角度生成
def positions_to_frames(pos, pos_gan, smoothed_2d, frame=0, xangle=0, is_upper2_body=False):
    logger.debug("角度計算 frame={0}".format(str(frame)))

	# 補正角度のクォータニオン
    decrease_correctqq = QQuaternion.fromEulerAngles(QVector3D(xangle * 0.5, 0, 0))
    increase_correctqq = QQuaternion.fromEulerAngles(QVector3D(xangle * 1.5, 0, 0))
    increase2_correctqq = QQuaternion.fromEulerAngles(QVector3D(xangle * 2, 0, 0))
    normal_correctqq = QQuaternion.fromEulerAngles(QVector3D(xangle, 0, 0))

    # 体幹の回転
    upper_body_rotation1, upper_body_rotation2, upper_correctqq, lower_body_rotation, lower_correctqq, is_gan \
        = positions_to_frames_trunk(frame, pos, pos_gan, is_upper2_body, decrease_correctqq, increase_correctqq, increase2_correctqq, normal_correctqq)

    # 上半身
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8f\xe3\x94\xbc\x90\x67' # '上半身'
    bf.rotation = upper_body_rotation1
    bone_frame_dic["上半身"].append(bf)

    # 上半身2(角度がある場合のみ登録)
    if upper_body_rotation2 != QQuaternion():
        bf = VmdBoneFrame(frame)
        bf.name = b'\x8f\xe3\x94\xbc\x90\x67\x32' # '上半身2'
        bf.rotation = upper_body_rotation2
        bone_frame_dic["上半身2"].append(bf)

    # 下半身
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\xba\x94\xbc\x90\x67' # '下半身'
    bf.rotation = lower_body_rotation
    bone_frame_dic["下半身"].append(bf)

    neck_rotation, head_rotation = \
        positions_to_frames_head(frame, pos, pos_gan, upper_body_rotation1, upper_body_rotation2, upper_correctqq, is_gan)

    # 首
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8e\xf1' # '首'
    bf.rotation = neck_rotation
    bone_frame_dic["首"].append(bf)

    # 頭
    bf = VmdBoneFrame(frame)
    bf.name = b'\x93\xaa' # '頭'
    bf.rotation = head_rotation
    bone_frame_dic["頭"].append(bf)

    # 左肩の初期値
    gan_left_shoulder_initial_orientation = QQuaternion.fromDirection(QVector3D(1, 0, 0), QVector3D(0, 1, 0))
    left_shoulder_initial_orientation = QQuaternion.fromDirection(QVector3D(2, -0.8, 0), QVector3D(0.5, -0.5, -1))
    left_arm_initial_orientation = QQuaternion.fromDirection(QVector3D(1.73, -1, 0), QVector3D(1, 1.73, 0))

    # 左手系の回転
    left_shoulder_rotation, left_arm_rotation, left_elbow_rotation = \
        positions_to_frames_arm_one_side(frame, pos, pos_gan, upper_correctqq, upper_body_rotation1, upper_body_rotation2, gan_left_shoulder_initial_orientation, left_shoulder_initial_orientation, left_arm_initial_orientation, LEFT_POINT, is_gan)

    # 左肩
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x8C\xA8' # '左肩'
    bf.rotation = left_shoulder_rotation
    bone_frame_dic["左肩"].append(bf)
    
    # 左腕
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x98\x72' # '左腕'
    bf.rotation = left_arm_rotation
    bone_frame_dic["左腕"].append(bf)
    
    # 左ひじ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x82\xd0\x82\xb6' # '左ひじ'
    bf.rotation = left_elbow_rotation
    bone_frame_dic["左ひじ"].append(bf)
    
    # 右肩の初期値
    gan_right_shoulder_initial_orientation = QQuaternion.fromDirection(QVector3D(-1, 0, 0), QVector3D(0, -1, 0))
    right_shoulder_initial_orientation = QQuaternion.fromDirection(QVector3D(-2, -0.8, 0), QVector3D(0.5, 0.5, 1))
    right_arm_initial_orientation = QQuaternion.fromDirection(QVector3D(-1.73, -1, 0), QVector3D(1, -1.73, 0))

    # 右手系の回転
    right_shoulder_rotation, right_arm_rotation, right_elbow_rotation = \
        positions_to_frames_arm_one_side(frame, pos, pos_gan, upper_correctqq, upper_body_rotation1, upper_body_rotation2, gan_right_shoulder_initial_orientation, right_shoulder_initial_orientation, right_arm_initial_orientation, RIGHT_POINT, is_gan)
    
    # 右肩
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x8C\xA8' # '右肩'
    bf.rotation = right_shoulder_rotation
    bone_frame_dic["右肩"].append(bf)
    
    # 右腕
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x98\x72' # '右腕'
    bf.rotation = right_arm_rotation
    bone_frame_dic["右腕"].append(bf)
    
    # 右ひじ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x82\xd0\x82\xb6' # '右ひじ'
    bf.rotation = right_elbow_rotation
    bone_frame_dic["右ひじ"].append(bf)

    # 左足と左ひざの回転
    left_leg_rotation, left_knee_rotation = \
        positions_to_frames_leg_one_side(frame, pos, pos_gan, lower_correctqq, lower_body_rotation, LEFT_POINT, ["左足", "左ひざ"], is_gan)

    # 左足
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x91\xab' # '左足'
    bf.rotation = left_leg_rotation
    bone_frame_dic["左足"].append(bf)
    
    # 左ひざ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x82\xd0\x82\xb4' # '左ひざ'
    bf.rotation = left_knee_rotation
    bone_frame_dic["左ひざ"].append(bf)

    # 右足と右ひざの回転
    right_leg_rotation, right_knee_rotation = \
        positions_to_frames_leg_one_side(frame, pos, pos_gan, lower_correctqq, lower_body_rotation, RIGHT_POINT, ["右足", "右ひざ"], is_gan)

    # 右足
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x91\xab' # '右足'
    bf.rotation = right_leg_rotation
    bone_frame_dic["右足"].append(bf)
    
    # 右ひざ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x82\xd0\x82\xb4' # '右ひざ'
    bf.rotation = right_knee_rotation
    bone_frame_dic["右ひざ"].append(bf)
        
    # センター(箱だけ作る)
    bf = VmdBoneFrame(frame)
    bf.name = b'\x83\x5A\x83\x93\x83\x5E\x81\x5B' # 'センター'
    bone_frame_dic["センター"].append(bf)

    # グルーブ(箱だけ作る)
    bf = VmdBoneFrame(frame)
    bf.name = b'\x83\x4F\x83\x8B\x81\x5B\x83\x75' # 'グルーブ'
    bone_frame_dic["グルーブ"].append(bf)

    # 左足ＩＫ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x91\xab\x82\x68\x82\x6a' # '左足ＩＫ'
    bone_frame_dic["左足ＩＫ"].append(bf)

    # 右足ＩＫ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x89\x45\x91\xab\x82\x68\x82\x6a' # '右足ＩＫ'
    bone_frame_dic["右足ＩＫ"].append(bf)

# 右系のpos point
RIGHT_POINT = {
    'Hip': 1,
    'Knee': 2,
    'Foot': 3,
    'Thorax': 8,
    'Shoulder': 14,
    'Elbow': 15,
    'Wrist': 16,
    'AnotherShoulder': 11
}
# 左系のpos point
LEFT_POINT = {
    'Hip': 4,
    'Knee': 5,
    'Foot': 6,
    'Thorax': 8,
    'Shoulder': 11,
    'Elbow': 12,
    'Wrist': 13,
    'AnotherShoulder': 14
}

def positions_to_frames_head(frame, pos, pos_gan, upper_body_rotation1, upper_body_rotation2, upper_correctqq, is_gan):
    if is_gan: 
        # 体幹が3dpose-ganで決定されている場合

        # 首
        direction = pos[9] - pos[8]
        up = QVector3D.crossProduct((pos[14] - pos[11]), direction).normalized()
        neck_orientation = QQuaternion.fromDirection(up, direction)
        initial_orientation = QQuaternion.fromDirection(QVector3D(0, 0, -1), QVector3D(0, 1, 0))
        rotation = neck_orientation * initial_orientation.inverted()
        neck_rotation = upper_body_rotation2.inverted() * upper_body_rotation1.inverted() * rotation

        # 頭
        direction = pos[10] - pos[9]
        up = QVector3D.crossProduct((pos[14] - pos[11]), (pos[10] - pos[9]))
        orientation = QQuaternion.fromDirection(direction, up)
        initial_orientation = QQuaternion.fromDirection(QVector3D(0, 1, 0), QVector3D(0, 0, 0))
        rotation = upper_correctqq * orientation * initial_orientation.inverted()
        head_rotation = neck_rotation.inverted() * upper_body_rotation2.inverted() * upper_body_rotation1.inverted() * rotation

    else:
        # 体幹が 3d-pose-baseline で決定されている場合

        # 首
        direction = pos[9] - pos[8]
        up = QVector3D.crossProduct((pos[14] - pos[11]), direction).normalized()
        neck_orientation = QQuaternion.fromDirection(up, direction)
        initial_orientation = QQuaternion.fromDirection(QVector3D(0, -1, 0), QVector3D(0, 0, -1))
        rotation = neck_orientation * initial_orientation.inverted()
        neck_rotation = upper_body_rotation2.inverted() * upper_body_rotation1.inverted() * rotation

        # 頭
        direction = pos[10] - pos[9]
        up = QVector3D.crossProduct((pos[9] - pos[8]), (pos[10] - pos[9]))
        orientation = QQuaternion.fromDirection(direction, up)
        initial_orientation = QQuaternion.fromDirection(QVector3D(0, 1, 0), QVector3D(1, 0, 0))
        rotation = upper_correctqq * orientation * initial_orientation.inverted()
        head_rotation = neck_rotation.inverted() * upper_body_rotation2.inverted() * upper_body_rotation1.inverted() * rotation

    return neck_rotation, head_rotation

def is_smoothed_prev_frame(frame, bone_rotation_dic, angle):
    # 最初は問答無用でOK
    if frame == 0:
        return True

    for bone_name, bone_rotation in bone_rotation_dic.items():    
        if len(bone_frame_dic[bone_name]) > frame-1:
            # 1F前の回転との差分をチェックする
            prev_euler = bone_frame_dic[bone_name][frame-1].rotation.toEulerAngles()
            now_euler = bone_rotation.toEulerAngles()

            if abs(prev_euler.x() - now_euler.x()) > angle \
                or abs(prev_euler.y() - now_euler.y()) > angle \
                or abs(prev_euler.z() - now_euler.z()) > angle :
                return False
    
    return True

def positions_to_frames_trunk(frame, pos, pos_gan, is_upper2_body, decrease_correctqq, increase_correctqq, increase2_correctqq, normal_correctqq):

    if pos_gan is not None:
        # 3dpose-gan による上半身FK
        g_upper_body_rotation1, g_upper_body_rotation2, g_upper_correctqq \
            = positions_to_frames_upper_calc(frame, pos_gan, is_upper2_body, QQuaternion(), QQuaternion(), QQuaternion(), QQuaternion())
        # 3dpose-gan による下半身FK
        g_lower_body_rotation, g_lower_correctqq \
            = positions_to_frames_lower_calc(frame, pos_gan, QQuaternion(), QQuaternion(), QQuaternion(), QQuaternion())

        # 前フレームとの差が45度以内で、オイラー角回転がどれか45度以上の場合、3dpose-gan採用
        # 体幹はY軸回転は見ない
        if is_smoothed_prev_frame(frame, { "上半身":g_upper_body_rotation1, "上半身2": g_upper_body_rotation2, "下半身":g_lower_body_rotation }, 45) \
            and (abs(g_upper_body_rotation1.toEulerAngles().x()) > 45 or abs(g_upper_body_rotation1.toEulerAngles().z()) > 45 \
                or abs(g_lower_body_rotation.toEulerAngles().x()) > 45 or abs(g_lower_body_rotation.toEulerAngles().z()) > 45 ):

            # # Zを反転させる
            # g_upper_body_rotation1.setZ( g_upper_body_rotation1.z() * -1 )
            # g_lower_body_rotation.setZ( g_lower_body_rotation.z() * -1 )
            
            logger.debug("gan採用: %s u1=%s u2=%s l=%s", frame, g_upper_body_rotation1, g_upper_body_rotation2, g_lower_body_rotation)

            return g_upper_body_rotation1, g_upper_body_rotation2, g_upper_correctqq, g_lower_body_rotation, g_lower_correctqq, True

    # 3d-pose-baseline による上半身FK
    upper_body_rotation1, upper_body_rotation2, upper_correctqq \
        = positions_to_frames_upper_calc(frame, pos, is_upper2_body, decrease_correctqq, increase_correctqq, increase2_correctqq, normal_correctqq)
        
    # 3d-pose-baseline による下半身FK
    lower_body_rotation, lower_correctqq \
        = positions_to_frames_lower_calc(frame, pos, decrease_correctqq, increase_correctqq, increase2_correctqq, normal_correctqq)

    return upper_body_rotation1, upper_body_rotation2, upper_correctqq, lower_body_rotation, lower_correctqq, False


# 上半身FK（実質計算用）
def positions_to_frames_upper_calc(frame, pos, is_upper2_body, decrease_correctqq, increase_correctqq, increase2_correctqq, normal_correctqq):

    if is_upper2_body == True:
        # 上半身2がある場合、分割して登録する

        # 上半身
        direction = pos[7] - pos[0]
        up = QVector3D.crossProduct(direction, (pos[14] - pos[11])).normalized()
        upper_body_orientation = QQuaternion.fromDirection(direction, up)
        initial = QQuaternion.fromDirection(QVector3D(0, 1, 0), QVector3D(0, 0, 1))
        upper_body_rotation1 = upper_body_orientation * initial.inverted()

        # 補正をかけて回転する
        if upper_body_rotation1.toEulerAngles().y() < 30 and upper_body_rotation1.toEulerAngles().y() > -30:
            # 前向きは増量補正
            upper_correctqq = increase_correctqq
        elif upper_body_rotation1.toEulerAngles().y() < -120 or upper_body_rotation1.toEulerAngles().y() > 120:
            # 後ろ向きは通常補正
            upper_correctqq = normal_correctqq
        else:
            # 横向きは減少補正
            upper_correctqq = decrease_correctqq

        upper_body_rotation1 = upper_body_rotation1 * upper_correctqq

        # 上半身2
        direction = pos[8] - pos[7]
        up = QVector3D.crossProduct(direction, (pos[14] - pos[11])).normalized()
        upper_body_orientation = QQuaternion.fromDirection(direction, up)
        initial = QQuaternion.fromDirection(QVector3D(0, 1, 0), QVector3D(0, 0, 1))
        upper_body_rotation2 = upper_body_orientation * initial.inverted()

        # 補正をかけて回転する
        if upper_body_rotation2.toEulerAngles().y() < 30 and upper_body_rotation2.toEulerAngles().y() > -30:
            # 前向きは増量大補正
            upper_correctqq = increase2_correctqq
        elif upper_body_rotation2.toEulerAngles().y() < -120 or upper_body_rotation2.toEulerAngles().y() > 120:
            # 後ろ向きは増量補正
            upper_correctqq = increase_correctqq
        else:
            # 横向きは通常補正
            upper_correctqq = normal_correctqq

        upper_body_rotation2 = upper_body_rotation1.inverted() * upper_body_rotation2 * upper_correctqq
        
    else:
        # 上半身2は初期クォータニオン
        upper_body_rotation2 = QQuaternion()
        
        """convert positions to bone frames"""
        # 上半身
        direction = pos[8] - pos[7]
        up = QVector3D.crossProduct(direction, (pos[14] - pos[11])).normalized()
        upper_body_orientation = QQuaternion.fromDirection(direction, up)
        initial = QQuaternion.fromDirection(QVector3D(0, 1, 0), QVector3D(0, 0, 1))
        upper_body_rotation1 = upper_body_orientation * initial.inverted()

        # 補正をかけて回転する
        if upper_body_rotation1.toEulerAngles().y() < 30 and upper_body_rotation1.toEulerAngles().y() > -30:
            # 前向きは増量補正
            upper_correctqq = increase_correctqq
        elif upper_body_rotation1.toEulerAngles().y() < -120 or upper_body_rotation1.toEulerAngles().y() > 120:
            # 後ろ向きは通常補正
            upper_correctqq = normal_correctqq
        else:
            # 横向きは減少補正
            upper_correctqq = decrease_correctqq

        upper_body_rotation1 = upper_body_rotation1 * upper_correctqq
           
    return upper_body_rotation1, upper_body_rotation2, upper_correctqq

# 下半身FK（実質計算用）
def positions_to_frames_lower_calc(frame, pos, decrease_correctqq, increase_correctqq, increase2_correctqq, normal_correctqq):
    direction = pos[0] - pos[7]
    up = QVector3D.crossProduct(direction, (pos[4] - pos[1]))
    lower_body_orientation = QQuaternion.fromDirection(direction, up)
    initial = QQuaternion.fromDirection(QVector3D(0, -1, 0), QVector3D(0, 0, 1))
    lower_body_rotation = lower_body_orientation * initial.inverted()

    # 補正をかけて回転する
    if lower_body_rotation.toEulerAngles().y() < 30 and lower_body_rotation.toEulerAngles().y() > -30:
        # 前向きは通常補正
        lower_correctqq = normal_correctqq
    elif lower_body_rotation.toEulerAngles().y() < -120 or lower_body_rotation.toEulerAngles().y() > 120:
        # 後ろ向きは通常補正
        lower_correctqq = normal_correctqq
    else:
        # 横向きは減少補正
        lower_correctqq = decrease_correctqq

    lower_body_rotation = lower_body_rotation * lower_correctqq

    return lower_body_rotation, lower_correctqq

# 片手のFK
def positions_to_frames_arm_one_side(frame, pos, pos_gan, upper_correctqq, upper_body_rotation1, upper_body_rotation2, gan_shoulder_initial_orientation, shoulder_initial_orientation, arm_initial_orientation, points, is_gan):

    if pos_gan is not None and is_gan == True:

        # 手(3dpose-gan採用)
        return positions_to_frames_shoulder_one_side_calc(frame, pos_gan, QQuaternion(), upper_body_rotation1, upper_body_rotation2, gan_shoulder_initial_orientation, arm_initial_orientation, points)

    # 3d-pose-baseline の手FK
    return positions_to_frames_shoulder_one_side_calc(frame, pos, upper_correctqq, upper_body_rotation1, upper_body_rotation2, shoulder_initial_orientation, arm_initial_orientation, points)


# 片方の手FKの実体
def positions_to_frames_shoulder_one_side_calc(frame, pos, upper_correctqq, upper_body_rotation1, upper_body_rotation2, shoulder_initial_orientation, arm_initial_orientation, points):

    # 肩
    direction = pos[points['Shoulder']] - pos[points['Thorax']]
    up = QVector3D.crossProduct((pos[points['Shoulder']] - pos[points['Thorax']]), (pos[points['AnotherShoulder']] - pos[points['Shoulder']]))
    orientation = QQuaternion.fromDirection(direction, up)
    rotation = upper_correctqq * orientation * shoulder_initial_orientation.inverted()
    # 肩ポーンの回転から親ボーンの回転を差し引いてbf.rotationに格納する。
    shoulder_rotation = upper_body_rotation2.inverted() * upper_body_rotation1.inverted() * rotation # 後で使うので保存しておく
    
    # 腕
    direction = pos[points['Elbow']] - pos[points['Shoulder']]
    up = QVector3D.crossProduct((pos[points['Elbow']] - pos[points['Shoulder']]), (pos[points['Wrist']] - pos[points['Elbow']]))
    orientation = QQuaternion.fromDirection(direction, up)
    rotation = upper_correctqq * orientation * arm_initial_orientation.inverted()
    # 腕ポーンの回転から親ボーンの回転を差し引いてbf.rotationに格納する。
    arm_rotation = shoulder_rotation.inverted() * upper_body_rotation2.inverted() * upper_body_rotation1.inverted() * rotation # 後で使うので保存しておく
    
    # ひじ
    direction = pos[points['Wrist']] - pos[points['Elbow']]
    up = QVector3D.crossProduct((pos[points['Elbow']] - pos[points['Shoulder']]), (pos[points['Wrist']] - pos[points['Elbow']]))
    orientation = QQuaternion.fromDirection(direction, up)
    rotation = upper_correctqq * orientation * arm_initial_orientation.inverted()
    # ひじポーンの回転から親ボーンの回転を差し引いてbf.rotationに格納する。
    # upper_body_rotation * left_shoulder_rotation * left_arm_rotation * bf.rotation = rotation なので、
    elbow_rotation = arm_rotation.inverted() * shoulder_rotation.inverted() * upper_body_rotation2.inverted() * upper_body_rotation1.inverted() * rotation
    # bf.rotation = (upper_body_rotation * left_arm_rotation).inverted() * rotation # 別の表現
    
    return shoulder_rotation, arm_rotation, elbow_rotation


# 片方の足のFK
def positions_to_frames_leg_one_side(frame, pos, pos_gan, lower_correctqq, lower_body_rotation, points, bone_names, is_gan):

    if pos_gan is not None:
        # 足(3dpose-gan採用)
        leg_rotation, knee_rotation = \
            positions_to_frames_leg_one_side_calc(frame, pos_gan, QQuaternion(), lower_body_rotation, points)

        # 体幹がgan採用もしくはオイラー角回転がどれか45度以上の場合、3dpose-gan採用
        if is_gan \
                or (abs(leg_rotation.toEulerAngles().x()) > 45 or abs(leg_rotation.toEulerAngles().y()) > 45 or abs(leg_rotation.toEulerAngles().z()) > 45 \
                or abs(knee_rotation.toEulerAngles().x()) > 45 or abs(knee_rotation.toEulerAngles().y()) > 45 or abs(knee_rotation.toEulerAngles().z()) > 45 ):
            return leg_rotation, knee_rotation
    
    # 3d-pose-baseline のFK
    leg_rotation, knee_rotation = \
        positions_to_frames_leg_one_side_calc(frame, pos, lower_correctqq, lower_body_rotation, points)

    # 足の角度が大人しい場合、3d-pose-baseline を採用
    return leg_rotation, knee_rotation

# 片方の足の実体
def positions_to_frames_leg_one_side_calc(frame, pos, lower_correctqq, lower_body_rotation, points):
    # 足
    direction = pos[points['Knee']] - pos[points['Hip']]
    up = QVector3D.crossProduct((pos[points['Knee']] - pos[points['Hip']]), (pos[points['Foot']] - pos[points['Knee']]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(0, -1, 0), QVector3D(-1, 0, 0))
    rotation = lower_correctqq * orientation * initial_orientation.inverted()
    leg_rotation = lower_body_rotation.inverted() * rotation
    
    # ひざ
    bf = VmdBoneFrame(frame)
    bf.name = b'\x8d\xb6\x82\xd0\x82\xb4' # 'ひざ'
    direction = pos[points['Foot']] - pos[points['Knee']]
    up = QVector3D.crossProduct((pos[points['Knee']] - pos[points['Hip']]), (pos[points['Foot']] - pos[points['Knee']]))
    orientation = QQuaternion.fromDirection(direction, up)
    initial_orientation = QQuaternion.fromDirection(QVector3D(0, -1, 0), QVector3D(-1, 0, 0))
    rotation = lower_correctqq * orientation * initial_orientation.inverted()
    knee_rotation = leg_rotation.inverted() * lower_body_rotation.inverted() * rotation

    return leg_rotation, knee_rotation


def read_positions(position_file):
    """Read joint position data"""
    f = open(position_file, "r")

    positions = []
    while True:
        line = f.readline()
        if not line:
            break
        line = line.rstrip('\r\n')
        a = re.split(' ', line)
        # 元データはz軸が垂直上向き。MMDに合わせるためにyとzを入れ替える。
        q = QVector3D(float(a[1]), float(a[3]), float(a[2])) # a[0]: index
        positions.append(q) # a[0]: index
    f.close()
    return positions

#複数フレームの読み込み
def read_positions_multi(position_file):
    """Read joint position data"""
    f = open(position_file, "r")

    positions = []
    while True:
        line = f.readline()
        if not line:
            break
        line = line.rstrip('\n')

        # 一旦カンマで複数行に分解
        inposition = []
        for inline in re.split(",\s*", line):
            if inline:
                # 1フレーム分に分解したら、空白で分解
                a = re.split(' ', inline)
                # print(a)
                # 元データはz軸が垂直上向き。MMDに合わせるためにyとzを入れ替える。
                q = QVector3D(float(a[1]), float(a[3]), float(a[2])) # a[0]: index
                inposition.append(q) # a[0]: index
        
        positions.append(inposition)
    f.close()
    return positions

def convert_position(pose_3d):
    positions = []
    for pose in pose_3d:
        for j in range(pose.shape[1]):
            q = QVector3D(pose[0, j], pose[2, j], pose[1, j])
            positions.append(q)
    return positions
    
# 開始フレームを取得
def load_start_frame(start_frame_file):
    n = 0
    with open(start_frame_file, "r") as sf:
        return int(sf.readline())

# 調整直立情報取得
def load_upright_target(upright_target):

    target_upright_idx = 0
    target_start_pos = {}
    if os.path.exists(upright_target):
        # 直立調整対象ファイルが存在する場合
        with open(upright_target +"/upright.txt", "r") as bf:
            # 直立IDX
            target_upright_idx = int(bf.readline())
            # 0Fセンターpos
            while True:
                s_line = bf.readline()

                if not s_line:
                    break

                # 直立の各数値取得
                poss = s_line.split(",")
                target_start_pos[poss[0]] = QVector3D()
                target_start_pos[poss[0]].setX(float(poss[1]))
                target_start_pos[poss[0]].setY(float(poss[2]))
                target_start_pos[poss[0]].setZ(float(poss[3]))
    else:
        # 直立調整対象ファイルが存在しない場合、初期値
        target_start_pos["center"] = QVector3D()
        for key in ["Neck", "RHip", "LHip", "RKnee", "LKnee", "RAnkle", "LAnkle"]:
            target_start_pos[key] = QVector3D()
            
    # logger.info("target_start_pos")
    # logger.info(target_start_pos)

    return target_upright_idx, target_start_pos
            
# 全身で最も直立している姿勢をいくつか返す
def calc_upright_body():
    return calc_upright_bones( ["上半身", "上半身2", "下半身", "左足", "左ひざ", "右足", "右ひざ"])

# 最も直立している姿勢をいくつか返す
def calc_upright_bones(target_bones):

    # ソート用に新しく辞書を生成する
    upright_bones_dic = {}

    keys =[]
    values =[]
    for n in range(len(bone_frame_dic[target_bones[0]])):
        keys.append(bone_frame_dic[target_bones[0]][n].frame)
        angles = []
        for bone_name in target_bones:
            if len(bone_frame_dic[bone_name]) > n:
                eular = bone_frame_dic[bone_name][n].rotation.toEulerAngles()
                angles.append(abs(eular.x()))
                angles.append(abs(eular.y()))
                angles.append(abs(eular.z()))
        values.append( np.nanmax(angles) )

    # logger.info(keys)
    # logger.info(values)
    
    upright_bones_dic = dict(zip(keys, values))

    # オイラー角の絶対最大値昇順でソートする。NaNは無視する
    sorted_upright_bones_dic = sorted(upright_bones_dic.items(), key=lambda x: x[1])

    logger.debug("ソート後")
    logger.debug(sorted_upright_bones_dic[:100])

    upright_idxs = []
    for k, v in sorted_upright_bones_dic[:100]:
        if is_almost_same_idx(upright_idxs, k, 30):
            continue
        
        upright_idxs.append(k)

        if len(upright_idxs) >= 10:
            break
            

    # # 直立に近い順のボーンリスト
    # upright_bones = [[0 for i in range(len(target_bones))] for j in range(100)]
    # for n, bone_name in enumerate(target_bones):
    #     # 直立昇順のインデックスリストを生成する
    #     for m, bone in enumerate(calc_upright_bone(bone_name)):
    #         # 配列は持ち方逆転
    #         # 0: 直立に近い順のインデックス
    #         # 1: ボーンインデックス
    #         upright_bones[m][n] = bone.frame

    # for n, bones_parts in enumerate(upright_bones[:3]):
    #     for m, b in enumerate(bones_parts[:5]):
    #         logger.debug("ソート前: {0} {1}: {2}".format(n, m, b))

    # upright_bones_flat = np.array(upright_bones_dic).flatten()

    # # 直立っぽいのを検出する
    # most_common_idxs = Counter(sorted_upright_bones_dic.values()).most_common()
    # logger.info(most_common_idxs)

    # upright_idxs = []
    # for most_common_idx in most_common_idxs:
    #     # 0フレーム目は除外
    #     if most_common_idx[0] != 0 and is_almost_same_idx(upright_idxs, most_common_idx[0], 30) == False:
    #         upright_idxs.append(most_common_idx[0])

    #     if len(upright_idxs) >= 10:
    #         break

    return upright_idxs

def calc_upright_key(bones):

    # 指定フレームの全指定ボーンの回転角度
    bone_rotations = []
    for k, v in bones.values():
        bone_rotations.append(v)
    
    return np.nanmax(np.array(bone_rotations))


# ほぼ同じようなインデックスの場合TRUE
def is_almost_same_idx(idxs, n, app):
    for i in idxs:
        if abs(i - n) < app:
            return True

    return False

def calc_upright_bone(bone_name):

    # ソート用に新しく配列を生成する
    upright_bones = []
    for bone in bone_frame_dic[bone_name]:
        upright_bones.append(copy.deepcopy(bone))

    logger.debug("ソート前: %s", bone_name)
    for n, b in enumerate(upright_bones[:10]):
        logger.debug("{0}: {1}, {2}, {3}".format(b.frame, b.rotation.x(), b.rotation.y(), b.rotation.z()))

    # オイラー角の絶対値合計値昇順でソートする。NaNは無視する
    upright_bones.sort(key=lambda x: np.nanmax(np.array([abs(x.rotation.toEulerAngles().x()), abs(x.rotation.toEulerAngles().y()), abs(x.rotation.toEulerAngles().z())]))) 

    logger.info("ソート後: %s", bone_name)
    for n, b in enumerate(upright_bones[:10]):
        logger.info("{0}: {1}, {2}, {3}".format(b.frame, b.rotation.toEulerAngles().x(), b.rotation.toEulerAngles().y(), b.rotation.toEulerAngles().z()))

    # # 1/300までのインデックスのみターゲットにする
    # upright_idxs = []
    # for n in range(round(len(bone_frame_dic[bone_name])/300)):
    #     upright_idxs.append(upright_bones[n])

    return upright_bones[:100]

# IKを滑らかにする
def smooth_IK(smooth_times):
    target_bones = ["左足ＩＫ", "右足ＩＫ"]

    # 関節の角度円滑化
    smooth_angle(smooth_times, target_bones)

    # 移動の位置円滑化
    smooth_move(smooth_times, target_bones)

# 回転を滑らかにする
def smooth_angle(smooth_times, target_bones):
    # 関節の角度円滑化
    for bone_name in target_bones:
        for n in range(smooth_times):
            for frame in range(len(bone_frame_dic[bone_name])):
                if frame >= 2:
                    prev2_bf = bone_frame_dic[bone_name][frame - 2]
                    prev1_bf = bone_frame_dic[bone_name][frame - 1]
                    now_bf = bone_frame_dic[bone_name][frame]

                    if prev2_bf != now_bf.rotation:
                        # 角度が違っていたら、球形補正開始
                        prev1_bf.rotation = QQuaternion.slerp(prev2_bf.rotation, now_bf.rotation, 0.5)


def smooth_move(smooth_times, target_bones):
    # センター移動の位置円滑化
    for bone_name in target_bones:
        for n in range(smooth_times):
            for frame in range(len(bone_frame_dic[bone_name])):
                if frame >= 4:
                    prev2_bf = bone_frame_dic[bone_name][frame - 2]
                    prev1_bf = bone_frame_dic[bone_name][frame - 1]
                    now_bf = bone_frame_dic[bone_name][frame]

                    # センターと足IKのどこかが動いていたら
                    if now_bf != prev2_bf:
                        # 線形補正
                        new_prev1_pos = prev2_bf.position + now_bf.position
                        new_prev1_pos /= 2
                        prev1_bf.position = new_prev1_pos

# センターボーンを間引きする
def decimate_bone_center_frames_array(base_dir, is_groove, mdecimation):

    base_center_x = []
    base_center_y = []
    base_center_z = []
    for bf in bone_frame_dic["センター"]:
        base_center_x.append(bf.position.x())
        base_center_y.append(bf.position.y())
        base_center_z.append(bf.position.z())

    base_groove_y = []
    for bf in bone_frame_dic["グルーブ"]:
        base_groove_y.append(bf.position.y())

    # フィットさせたフレーム情報
    fit_frames = []
    fit_center_x_points = []
    fit_center_y_points = []
    fit_center_z_points = []
    fit_groove_y_points = []

    for n in range(len(base_center_x)):
        logger.debug("decimate_bone_center_frames_array n={0} ------------- ".format(n))
        
        # 最初は固定登録
        if n == 0:
            fit_frames.append(n)
            fit_center_x_points.append(base_center_x[n])
            fit_center_y_points.append(base_center_y[n])
            fit_center_z_points.append(base_center_z[n])
            fit_groove_y_points.append(base_groove_y[n])
            continue
            
        # 前のキーフレームのXY（2点）
        prev_x1 = np.array(fit_center_x_points[-2:])
        prev_y1 = np.array(fit_center_y_points[-2:])
        prev_z1 = np.array(fit_center_z_points[-2:])

        # logger.debug("prev_x1")
        # logger.debug(prev_x1)
        # logger.debug("prev_y1")
        # logger.debug(prev_y1)
        # logger.debug("prev_z1")
        # logger.debug(prev_z1)
        
        # 登録対象キーフレームのXY
        now_x1 = np.array(base_center_x[0+n:3+n])
        now_y1 = np.array(base_center_y[0+n:3+n])
        now_z1 = np.array(base_center_z[0+n:3+n])
        
        # 登録対象キーフレームのXY
        now_long_x1 = np.array(base_center_x[0+n:6+n])
        now_long_y1 = np.array(base_center_y[0+n:6+n])
        now_long_z1 = np.array(base_center_z[0+n:6+n])
        
        # logger.debug("now_x1")
        # logger.debug(now_x1)
        # logger.debug("now_y1")
        # logger.debug(now_y1)
        # logger.debug("now_z1")
        # logger.debug(now_z1)

        # 次回登録対象キーフレームのXY
        next_x1 = np.array(base_center_x[3+n:6+n])
        next_y1 = np.array(base_center_y[3+n:6+n])
        next_z1 = np.array(base_center_z[3+n:6+n])

        # 前のキーフレームの近似直線(XY)
        if len(prev_x1) <= 1 or ( np.average(prev_x1) == prev_x1[0] and np.average(prev_y1) == prev_y1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0
            prev_poly_xy_angle = 0
        else:
            prev_poly_xy_fit1 = np.polyfit(prev_x1, prev_y1, 1)
            # prev_poly_xy_fit1_y_func = np.poly1d(prev_poly_xy_fit1)
            # 近似直線の角度
            prev_poly_xy_angle = np.rad2deg(np.arctan(prev_poly_xy_fit1[0]))

        # logger.debug("prev_poly_xy_angle")
        # logger.debug(prev_poly_xy_angle)
        
        if len(now_x1) <= 1 or ( np.average(now_x1) == now_x1[0] and np.average(now_y1) == now_y1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0
            now_poly_xy_angle = 0
        else:
            # 登録対象キーフレームの近似直線(XY)
            now_poly_xy_fit1 = np.polyfit(now_x1, now_y1, 1)
            # now_poly_xy_fit1_y_func = np.poly1d(now_poly_xy_fit1)
            # 近似直線の角度
            now_poly_xy_angle = np.rad2deg(np.arctan(now_poly_xy_fit1[0]))

        # logger.debug("now_poly_xy_angle")
        # logger.debug(now_poly_xy_angle)

        if len(now_long_x1) <= 1 or ( np.average(now_long_x1) == now_long_x1[0] and np.average(now_long_y1) == now_long_y1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0
            now_long_poly_xy_angle = 0
        else:
            # 登録対象長めキーフレームの近似直線(XY)
            now_long_poly_xy_fit1 = np.polyfit(now_long_x1, now_long_y1, 1)
            # now_long_poly_xy_fit1_y_func = np.poly1d(now_long_poly_xy_fit1)
            # 近似直線の角度
            now_long_poly_xy_angle = np.rad2deg(np.arctan(now_long_poly_xy_fit1[0]))

        # logger.debug("now_long_poly_xy_angle")
        # logger.debug(now_long_poly_xy_angle)

        # 次のキーフレームの近似直線(XY)
        if len(next_x1) <= 1 or ( np.average(next_x1) == next_x1[0] and np.average(next_y1) == next_y1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0
            next_poly_xy_angle = 0
        else:
            next_poly_xy_fit1 = np.polyfit(next_x1, next_y1, 1)
            # next_poly_xy_fit1_y_func = np.poly1d(next_poly_xy_fit1)
            # 近似直線の角度
            next_poly_xy_angle = np.rad2deg(np.arctan(next_poly_xy_fit1[0]))

        # 角度差分
        diff_prev_xy = abs(np.diff([prev_poly_xy_angle, now_poly_xy_angle]))
        diff_long_xy = abs(np.diff([now_long_poly_xy_angle, now_poly_xy_angle]))
        diff_next_xy = abs(np.diff([next_poly_xy_angle, now_poly_xy_angle]))

        if len(prev_x1) <= 1 or ( np.average(prev_x1) == prev_x1[0] and np.average(prev_z1) == prev_z1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
            prev_poly_xz_angle = 0
        else:
            # 前のキーフレームの近似直線(XZ)
            prev_poly_xz_fit1 = np.polyfit(prev_x1, prev_z1, 1)
            # prev_poly_xz_fit1_y_func = np.poly1d(prev_poly_xz_fit1)
            # 近似直線の角度
            prev_poly_xz_angle = np.rad2deg(np.arctan(prev_poly_xz_fit1[0]))

        # logger.debug("prev_poly_xz_angle")
        # logger.debug(prev_poly_xz_angle)
    
        if len(now_x1) <= 1 or ( np.average(now_x1) == now_x1[0] and np.average(now_z1) == now_z1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
            now_poly_xz_angle = 0
        else:
            # 登録対象キーフレームの近似直線(XZ)
            now_poly_xz_fit1 = np.polyfit(now_x1, now_z1, 1)
            # now_poly_xz_fit1_y_func = np.poly1d(now_poly_xz_fit1)
            # 近似直線の角度
            now_poly_xz_angle = np.rad2deg(np.arctan(now_poly_xz_fit1[0]))

        # logger.debug("now_poly_xz_angle")
        # logger.debug(now_poly_xz_angle)
    
        if len(now_long_x1) <= 1 or ( np.average(now_long_x1) == now_long_x1[0] and np.average(now_long_z1) == now_long_z1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
            now_long_poly_xz_angle = 0
        else:
            # 登録対象長めキーフレームの近似直線(XZ)
            now_long_poly_xz_fit1 = np.polyfit(now_long_x1, now_long_z1, 1)
            # now_long_poly_xz_fit1_y_func = np.poly1d(now_long_poly_xz_fit1)
            # 近似直線の角度
            now_long_poly_xz_angle = np.rad2deg(np.arctan(now_long_poly_xz_fit1[0]))

        if len(next_x1) <= 1 or ( np.average(next_x1) == next_x1[0] and np.average(next_z1) == next_z1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
            next_poly_xz_angle = 0
        else:
            # 前のキーフレームの近似直線(XZ)
            next_poly_xz_fit1 = np.polyfit(next_x1, next_z1, 1)
            # next_poly_xz_fit1_y_func = np.poly1d(next_poly_xz_fit1)
            # 近似直線の角度
            next_poly_xz_angle = np.rad2deg(np.arctan(next_poly_xz_fit1[0]))

        # logger.debug("now_long_poly_xz_angle")
        # logger.debug(now_long_poly_xz_angle)

        # 角度差分
        diff_prev_xz = abs(np.diff([prev_poly_xz_angle, now_poly_xz_angle]))
        diff_long_xz = abs(np.diff([now_long_poly_xz_angle, now_poly_xz_angle]))
        diff_next_xz = abs(np.diff([next_poly_xz_angle, now_poly_xz_angle]))
                
    
        if (( diff_prev_xy > 15 and diff_long_xy > 15) or (diff_prev_xz > 15 and diff_long_xz > 15)) \
            and (abs(np.diff([fit_center_x_points[-1], base_center_x[n]])) >= mdecimation \
                or abs(np.diff([fit_center_y_points[-1], base_center_y[n]])) >= mdecimation \
                or abs(np.diff([fit_center_z_points[-1], base_center_z[n]])) >= mdecimation) :
            # 前回と今回の角度の差が規定より大きい場合、キーフレーム登録
            fit_frames.append(n)
            fit_center_x_points.append(base_center_x[n])
            fit_center_y_points.append(base_center_y[n])
            fit_center_z_points.append(base_center_z[n])
            fit_groove_y_points.append(base_groove_y[n])
        elif len(base_center_x) > n + 1 \
            and (( diff_next_xy > 15 and diff_long_xy > 15 ) or ( diff_next_xz > 15 and diff_long_xz > 15 )) \
            and (abs(np.diff([fit_center_x_points[-1], base_center_x[n]])) < mdecimation \
                or abs(np.diff([fit_center_y_points[-1], base_center_y[n]])) < mdecimation \
                or abs(np.diff([fit_center_z_points[-1], base_center_z[n]])) < mdecimation) \
            and (abs(np.diff([fit_center_x_points[-1], base_center_x[n+1]])) >= mdecimation \
                or abs(np.diff([fit_center_y_points[-1], base_center_y[n+1]])) >= mdecimation \
                or abs(np.diff([fit_center_z_points[-1], base_center_z[n+1]])) >= mdecimation) :
            # 今回と次回の角度の差が規定より大きい場合、タメとしてキーフレーム登録
            fit_frames.append(n)
            # fit_center_x_points.append(fit_center_x_points[-1])
            # fit_center_y_points.append(fit_center_y_points[-1])
            # fit_center_z_points.append(fit_center_z_points[-1])
            # fit_groove_y_points.append(fit_groove_y_points[-1])
            fit_center_x_points.append(base_center_x[n])
            fit_center_y_points.append(base_center_y[n])
            fit_center_z_points.append(base_center_z[n])
            fit_groove_y_points.append(base_groove_y[n])
        elif is_groove and is_regist_groove_frame(n, base_groove_y, fit_frames, fit_groove_y_points, mdecimation):
            # 前回と今回の角度の差が規定より小さく、かつグルーブがある場合、追加判定する
            fit_frames.append(n)
            fit_center_x_points.append(base_center_x[n])
            fit_center_y_points.append(base_center_y[n])
            fit_center_z_points.append(base_center_z[n])
            fit_groove_y_points.append(base_groove_y[n])
        else:
            # グルーブがない場合はスルー
            pass                

    logger.debug(len(fit_center_x_points))

    # センター・グルーブを再登録
    center_newbfs = []
    groove_newbfs = []

    for nidx, n in enumerate(fit_frames) :
        # 一旦オリジナルをコピー
        center_bf = bone_frame_dic["センター"][n]
        center_bf.position.setX(fit_center_x_points[nidx])
        center_bf.position.setY(fit_center_y_points[nidx])
        center_bf.position.setZ(fit_center_z_points[nidx])
        # 整形したデータを登録
        center_newbfs.append(center_bf)

        if is_groove:
            # 一旦オリジナルをコピー
            groove_bf = bone_frame_dic["グルーブ"][n]
            groove_bf.position.setY(fit_groove_y_points[nidx])
            # 整形したデータを登録
            groove_newbfs.append(groove_bf)

    bone_frame_dic["センター"] = center_newbfs
    bone_frame_dic["グルーブ"] = groove_newbfs

    # if level[verbose] == logging.DEBUG:
    #     fit_start = 0
    #     for n in range(0, len(base_x), 50):
    #         plt.cla()
    #         plt.clf()
    #         fig, (axY, axZ) = plt.subplots(ncols=2, figsize=(15, 15))
    #         axY.plot(np.array(base_x[n:n+50]) , np.array(base_y[n:n+50]), lw=2)
    #         axZ.plot(np.array(base_x[n:n+50]) , np.array(base_z[n:n+50]), lw=2)

    #         fit_end = 0
    #         for m in range(fit_start, len(fit_frames)):
    #             if fit_frames[m] >= n+50:
    #                 fit_end = m
    #                 break

    #         axY.plot(np.array(fit_x_points[fit_start:fit_end]) , np.array(fit_y_points[fit_start:fit_end]), c="#CC0000")
    #         axZ.plot(np.array(fit_x_points[fit_start:fit_end]) , np.array(fit_z_points[fit_start:fit_end]), c="#CC0000")

    #         fit_start = fit_end + 1

    #         plotName = "{0}/plot_{1}_{2:05d}.png".format(base_dir, bone_name, n)
    #         plt.savefig(plotName)
    #         plt.close()

# グルーブボーンを登録するか否か判定する
def is_regist_groove_frame(n, base_y, fit_frames, fit_y_points, mdecimation):
    # 2つ前のキーフレームのY（1点）
    if len(fit_y_points) == 1:
        prev_y2 = fit_y_points[-1]
    else:
        prev_y2 = fit_y_points[-2]

    # logger.debug("prev_y2")
    # logger.debug(prev_y2)
        
    # 前のキーフレームのY（1点）
    prev_y1 = fit_y_points[-1]

    # logger.debug("prev_y1")
    # logger.debug(prev_y1)

    # 次のキーフレームのY
    if len(base_y) - 1 == n:
        next_y1 = base_y[n]
    else:
        next_y1 = base_y[n + 1]

    # logger.debug("next_y1")
    # logger.debug(next_y1)
    
    # if n > 2450 and n < 2500:
    #     logger.debug("base_y[n]: {0}, prev_y1: {1}, base_y[n] - prev_y1: {2}".format(base_y[n], prev_y1, base_y[n] - prev_y1))
    #     logger.debug("prev_y1: {0}, prev_y2: {1}, prev_y1 - prev_y2: {2}".format(prev_y1, prev_y2, prev_y1 - prev_y2))
    #     logger.debug("base_y[n]: {0}, prev_y1: {1}, base_y[n] - prev_y1: {2}".format(base_y[n], prev_y1, base_y[n] - prev_y1))
    #     logger.debug("prev_y1: {0}, prev_y2: {1}, prev_y1 - prev_y2: {2}".format(prev_y1, prev_y2, prev_y1 - prev_y2))
    #     logger.debug("abs(np.diff([prev_y1, base_y[n]])): {0}".format(abs(np.diff([prev_y1, base_y[n]]))))

    if (base_y[n] - prev_y1 < 0 and prev_y1 - prev_y2 >= 0 and abs(np.diff([prev_y1, base_y[n]])) >= mdecimation ) \
        or (base_y[n] - prev_y1 > 0 and prev_y1 - prev_y2 <= 0 and abs(np.diff([prev_y1, base_y[n]])) >= mdecimation ) :
        # 前回の移動と符号が反転している（上下逆に運動している）場合
        # かつ移動量が規定量以上の場合、登録
        return True
    elif (next_y1 - base_y[n] < 0 and base_y[n] - prev_y1 >= 0 and abs(np.diff([next_y1, base_y[n]])) >= mdecimation ) \
        or (next_y1 - base_y[n] > 0 and base_y[n] - prev_y1 <= 0 and abs(np.diff([next_y1, base_y[n]])) >= mdecimation ) :
        # 次の移動と符号が反転している（上下逆に運動している）場合
        # かつ移動量が規定量以上の場合、登録
        # タメ登録
        return True
    else:
        # 前回と今回の角度の差が規定より小さい場合、スルー
        return False

# 回転ボーンを揃えて登録する
def decimate_bone_rotation_frames_array(bone_name_array, ddecimation):

    # フィットさせたフレーム情報
    fit_frames = []

    # 同じフレーム内で回す
    for n in range(len(bone_frame_dic[bone_name_array[0]])):
        logger.debug("decimate_bone_rotation_frames_array n={0} ------------- ".format(n))

        if n == 0:
            # 最初は必ず登録
            fit_frames.append(n)
            continue
            
        for bone_name in bone_name_array:
            if is_regist_rotation_frame(n, bone_name, fit_frames, ddecimation):
                # 登録可能な場合、登録
                fit_frames.append(n)
                break
        
    # 登録する場合、全ボーンの同じフレームで登録する
    for bone_name in bone_name_array:
        newbfs = []
        for n in fit_frames:
            # logger.debug("n={0}, bone_name={1}, len={2}".format(n, bone_name, len(bone_frame_dic[bone_name])))
            newbfs.append(bone_frame_dic[bone_name][n])
        
        # 新しいフレームリストを登録する
        bone_frame_dic[bone_name] = newbfs



# 回転系のフレームを登録するか否か判定する
def is_regist_rotation_frame(n, bone_name, fit_frames, ddecimation):
    # # 二つ前の回転情報
    # if len(fit_frames) == 1:
    #     prev2_rotation = bone_frame_dic[bone_name][fit_frames[-1]].rotation
    # else:
    #     prev2_rotation = bone_frame_dic[bone_name][fit_frames[-2]].rotation

    # logger.debug("prev2_rotation x={0}, y={1}, z={2}, scalar={3}".format(prev2_rotation.x(), prev2_rotation.y(), prev2_rotation.z(), prev2_rotation.scalar()))

    # 一つ前の回転情報
    prev1_rotation = bone_frame_dic[bone_name][fit_frames[-1]].rotation

    logger.debug("prev1_rotation.euler x={0}, y={1}, z={2}".format(prev1_rotation.toEulerAngles().x(), prev1_rotation.toEulerAngles().y(), prev1_rotation.toEulerAngles().z()))

    # 今回判定対象の回転情報
    now_rotation = bone_frame_dic[bone_name][n].rotation

    logger.debug("now_rotation.euler x={0}, y={1}, z={2}".format(now_rotation.toEulerAngles().x(), now_rotation.toEulerAngles().y(), now_rotation.toEulerAngles().z()))

    # # 2つ前から前回までの回転の差
    # diff_prev_rotation = QQuaternion.rotationTo(prev2_rotation.normalized().vector(), prev1_rotation.normalized().vector())

    # logger.debug("diff_prev_rotation x={0}, y={1}, z={2}, scalar={3}".format(diff_prev_rotation.x(), diff_prev_rotation.y(), diff_prev_rotation.z(), diff_prev_rotation.scalar()))
    # logger.debug("diff_prev_rotation.euler x={0}, y={1}, z={2}".format(diff_prev_rotation.toEulerAngles().x(), diff_prev_rotation.toEulerAngles().y(), diff_prev_rotation.toEulerAngles().z()))

    # diff_prev_ary = [diff_prev_rotation.x(), diff_prev_rotation.y(), diff_prev_rotation.z()]
    # diff_prev_max_index = np.argmax(diff_prev_ary)

    # logger.debug("diff_prev_max_index={0}".format(diff_prev_max_index))

    # diff_now_rotation = QQuaternion.rotationTo(prev1_rotation.normalized().vector(), now_rotation.normalized().vector())

    # logger.debug("diff_now_rotation x={0}, y={1}, z={2}, scalar={3}".format(diff_now_rotation.x(), diff_now_rotation.y(), diff_now_rotation.z(), diff_now_rotation.scalar()))
    # logger.debug("diff_now_rotation.euler x={0}, y={1}, z={2}".format(diff_now_rotation.toEulerAngles().x(), diff_now_rotation.toEulerAngles().y(), diff_now_rotation.toEulerAngles().z()))

    # diff_now_rotation2 = now_rotation - prev1_rotation

    # logger.debug("diff_now_rotation2 x={0}, y={1}, z={2}, scalar={3}".format(diff_now_rotation2.x(), diff_now_rotation2.y(), diff_now_rotation2.z(), diff_now_rotation2.scalar()))
    # logger.debug("diff_now_rotation2.euler x={0}, y={1}, z={2}".format(diff_now_rotation2.toEulerAngles().x(), diff_now_rotation2.toEulerAngles().y(), diff_now_rotation2.toEulerAngles().z()))

    # 前回から今回までの回転の差
    diff_now_rotation3 = prev1_rotation.inverted() * now_rotation

    logger.debug("diff_now_rotation3 x={0}, y={1}, z={2}, scalar={3}".format(diff_now_rotation3.x(), diff_now_rotation3.y(), diff_now_rotation3.z(), diff_now_rotation3.scalar()))
    logger.debug("diff_now_rotation3.euler x={0}, y={1}, z={2}".format(diff_now_rotation3.toEulerAngles().x(), diff_now_rotation3.toEulerAngles().y(), diff_now_rotation3.toEulerAngles().z()))

    # 一定以上回転していれば登録対象
    if abs(diff_now_rotation3.toEulerAngles().x()) >= ddecimation or abs(diff_now_rotation3.toEulerAngles().y()) >= ddecimation or abs(diff_now_rotation3.toEulerAngles().z()) >= ddecimation :
        if bone_name == "上半身" or bone_name == "下半身":
            # 細かい動きをするボーンは、1F隣でもOKとする
            return True
        elif fit_frames[-1] + 1 < n:
            return True
    elif diff_now_rotation3.toEulerAngles().y() < 0 and ( now_rotation.toEulerAngles().y() <= -90 or now_rotation.toEulerAngles().y() >= 90 ):
        # 角度が小さくても後ろ向きなら登録対象
        if bone_name == "上半身" or bone_name == "下半身":
            return True

    # 次回判定対象の回転情報
    if len(bone_frame_dic[bone_name]) > n + 1:
        next_rotation = bone_frame_dic[bone_name][n+1].rotation

        logger.debug("next_rotation.euler x={0}, y={1}, z={2}".format(next_rotation.toEulerAngles().x(), next_rotation.toEulerAngles().y(), next_rotation.toEulerAngles().z()))

        # 今回から次回までの回転の差
        diff_next_rotation3 = now_rotation.inverted() * next_rotation

        logger.debug("diff_next_rotation3 x={0}, y={1}, z={2}, scalar={3}".format(diff_next_rotation3.x(), diff_next_rotation3.y(), diff_next_rotation3.z(), diff_next_rotation3.scalar()))
        logger.debug("diff_next_rotation3.euler x={0}, y={1}, z={2}".format(diff_next_rotation3.toEulerAngles().x(), diff_next_rotation3.toEulerAngles().y(), diff_next_rotation3.toEulerAngles().z()))

        # 次回一定以上回転するようであれば、今回登録対象
        if abs(diff_next_rotation3.toEulerAngles().x()) >= ddecimation or abs(diff_next_rotation3.toEulerAngles().y()) >= ddecimation or abs(diff_next_rotation3.toEulerAngles().z()) >= ddecimation :
            if bone_name == "上半身" or bone_name == "下半身":
                # 細かい動きをするボーンは、1F隣でもOKとする
                return True
            elif fit_frames[-1] + 1 < n:
                return True        

    return False


# IKの計算
def calc_IK(bone_csv_file, smoothed_2d, depth_all_frames, upright_idxs, heelpos):
    logger.debug("bone_csv_file: "+ bone_csv_file)

    upright_idx = upright_idxs[0]

    # ボーンファイルを開く
    with open(bone_csv_file, "r", encoding=get_file_encoding(bone_csv_file)) as bf:
        reader = csv.reader(bf)

        for row in reader:

            if row[1] == "下半身" or row[2].lower() == "lower body":
                # 下半身ボーン
                lower_body_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左足" or row[2].lower() == "leg_l":
                # 左足ボーン
                left_leg_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左ひざ" or row[2].lower() == "knee_l":
                # 左ひざボーン
                left_knee_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左足首" or row[2].lower() == "ankle_l":
                # 左足首ボーン
                left_ankle_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左つま先" or row[2].lower() == "l toe":
                # 左つま先ボーン
                left_toes_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右足" or row[2].lower() == "leg_r":
                # 右足ボーン
                right_leg_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右ひざ" or row[2].lower() == "knee_r":
                # 右ひざボーン
                right_knee_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右足首" or row[2].lower() == "ankle_r":
                # 右足首ボーン
                right_ankle_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右つま先" or row[2].lower() == "r toe":
                # 右つま先ボーン
                right_toes_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[0] == "Bone" and (row[1] == "左足ＩＫ" or row[2].lower() == "leg ik_l"):
                # 左足ＩＫボーン
                left_leg_ik_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[0] == "Bone" and (row[1] == "右足ＩＫ" or row[2].lower() == "leg ik_r"):
                # 右足ＩＫボーン
                right_leg_ik_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "センター" or row[2].lower() == "center":
                # センターボーン
                center_bone = QVector3D(float(row[5]), float(row[6]), float(row[7]))

    # 2Dの直立フレームの腰の位置
    center_upright_2d_y = (smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["RHip"]].y() + smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["LHip"]].y()) / 2

    # 前回フレーム
    prev_left_frame = 0
    prev_right_frame = 0

    for n in range(len(bone_frame_dic["左足"])):
        logger.debug("足IK計算 frame={0}".format(n))
        # logger.debug("右足踵={0}, 左足踵={1}".format(smoothed_2d[n][SMOOTHED_2D_INDEX["RAnkle"]], smoothed_2d[n][SMOOTHED_2D_INDEX["LAnkle"]]))

        # logger.debug("前回左x={0}, 今回左x={1}, 差分={2}".format(smoothed_2d[prev_left_frame][4].x(), smoothed_2d[n][SMOOTHED_2D_INDEX["LAnkle"]].x(), abs(np.diff([smoothed_2d[prev_left_frame][4].x(), smoothed_2d[n][SMOOTHED_2D_INDEX["LAnkle"]].x()]))))
        # logger.debug("前回左y={0}, 今回左y={1}, 差分={2}".format(smoothed_2d[prev_left_frame][4].y(), smoothed_2d[n][SMOOTHED_2D_INDEX["LAnkle"]].y(), abs(np.diff([smoothed_2d[prev_left_frame][4].y(), smoothed_2d[n][SMOOTHED_2D_INDEX["LAnkle"]].y()]))))

        #左足IK
        if n > 0 and abs(np.diff([smoothed_2d[prev_left_frame][SMOOTHED_2D_INDEX["LAnkle"]].x(), smoothed_2d[n][SMOOTHED_2D_INDEX["LAnkle"]].x()])) < 5 and abs(np.diff([smoothed_2d[prev_left_frame][SMOOTHED_2D_INDEX["LAnkle"]].y(), smoothed_2d[n][SMOOTHED_2D_INDEX["LAnkle"]].y()])) < 5:
            # ほぼ動いていない場合、前回分をコピー
            # logger.debug("前回左IKコピー")

            # 前回からほぼ動いていない場合、前回の値をコピーする
            left_ankle_pos = bone_frame_dic["左足ＩＫ"][prev_left_frame].position
            left_ik_rotation = bone_frame_dic["左足ＩＫ"][prev_left_frame].rotation
            left_leg_diff_rotation = bone_frame_dic["左足"][prev_left_frame].rotation
        else:
            # 前回から動いている場合、計算する
            # 左足IK
            (left_ankle_pos, left_ik_rotation, left_leg_diff_rotation) = \
                calc_IK_matrix(center_bone, lower_body_bone, left_leg_bone, left_knee_bone, left_ankle_bone, left_toes_bone, left_leg_ik_bone \
                    , bone_frame_dic["センター"][n].position \
                    , bone_frame_dic["下半身"][n].rotation, bone_frame_dic["左足"][n].rotation, bone_frame_dic["左ひざ"][n].rotation )

            # 前回登録フレームとして保持
            prev_left_frame = n

        # logger.debug("前回右x={0}, 今回右x={1}, 差分={2}".format(smoothed_2d[prev_left_frame][3].x(), smoothed_2d[n][SMOOTHED_2D_INDEX["RAnkle"]].x(), abs(np.diff([smoothed_2d[prev_left_frame][3].x(), smoothed_2d[n][SMOOTHED_2D_INDEX["RAnkle"]].x()]))))
        # logger.debug("前回右y={0}, 今回右y={1}, 差分={2}".format(smoothed_2d[prev_left_frame][3].y(), smoothed_2d[n][SMOOTHED_2D_INDEX["RAnkle"]].y(), abs(np.diff([smoothed_2d[prev_left_frame][3].y(), smoothed_2d[n][SMOOTHED_2D_INDEX["RAnkle"]].y()]))))
            
        # 右足IK
        if n > 0 and abs(np.diff([smoothed_2d[prev_right_frame][SMOOTHED_2D_INDEX["RAnkle"]].x(), smoothed_2d[n][SMOOTHED_2D_INDEX["RAnkle"]].x()])) < 5 and abs(np.diff([smoothed_2d[prev_right_frame][SMOOTHED_2D_INDEX["RAnkle"]].y(), smoothed_2d[n][SMOOTHED_2D_INDEX["RAnkle"]].y()])) < 5:
            # ほぼ動いていない場合、前回分をコピー
            # logger.debug("前回右IKコピー")

            right_ankle_pos = bone_frame_dic["右足ＩＫ"][prev_right_frame].position
            right_ik_rotation = bone_frame_dic["右足ＩＫ"][prev_right_frame].rotation
            right_leg_diff_rotation = bone_frame_dic["右足"][prev_right_frame].rotation
        else:          
            # 右足IK
            (right_ankle_pos, right_ik_rotation, right_leg_diff_rotation) = \
                calc_IK_matrix(center_bone, lower_body_bone, right_leg_bone, right_knee_bone, right_ankle_bone, right_toes_bone, right_leg_ik_bone \
                    , bone_frame_dic["センター"][n].position \
                    , bone_frame_dic["下半身"][n].rotation, bone_frame_dic["右足"][n].rotation, bone_frame_dic["右ひざ"][n].rotation )
            
            # 前回登録フレームとして保持
            prev_right_frame = n

        # 右足も左足も計算しなかった場合
        if n > 0 and prev_left_frame != n and prev_right_frame != n:
            # 前回インデックスで近い方採用
            prev_idx = prev_left_frame if prev_left_frame > prev_right_frame else prev_right_frame
            # センターZを動かさない
            # logger.info("n=%s, previdx=%s", n, prev_idx)
            # logger.info("z変更前=%s", bone_frame_dic["センター"][n].position.z())
            bone_frame_dic["センター"][n].position.setZ( bone_frame_dic["センター"][prev_idx].position.z() )
            # logger.info("z変更後=%s", bone_frame_dic["センター"][n].position.z())


        # 右足も左足も計算した場合
        # if prev_left_frame == prev_right_frame == n:

            # if heelpos != 0:
            #     # 踵位置補正がかかっている場合、補正を加算する
            #     left_ankle_pos.setY(left_ankle_pos.y() + heelpos)
            #     right_ankle_pos.setY(right_ankle_pos.y() + heelpos)
            #     bone_frame_dic["センター"][n].position.setY( bone_frame_dic["センター"][n].position.y() + heelpos )
        # elif prev_left_frame != n and prev_right_frame != n:
        #     # 固定位置が近い方のINDEXを取得する
        #     prev_idx = prev_left_frame if prev_left_frame <= prev_right_frame else prev_right_frame

        #     # 右足も左足も計算しなかった場合、センターをコピーする
        #     bone_frame_dic["センター"][n].position.setY( bone_frame_dic["センター"][prev_idx].position.y() )

        # logger.debug("left_ankle_pos:{0}, right_ankle_pos: {1}".format(left_ankle_pos, right_ankle_pos))

        # 両足IKがマイナスの場合(地面にめり込んでいる場合)
        if left_ankle_pos.y() < 0 and right_ankle_pos.y() < 0:
            ankle_pos_max = np.max([left_ankle_pos.y(), right_ankle_pos.y()])

            # logger.debug("ankle_pos_max:{0}".format(ankle_pos_max))    

            # logger.debug("center.y1:{0}".format(bone_frame_dic["センター"][n].position.y()))    

            # センターも一緒にあげる
            left_ankle_pos.setY( left_ankle_pos.y() - ankle_pos_max )
            right_ankle_pos.setY( right_ankle_pos.y() - ankle_pos_max )
            bone_frame_dic["センター"][n].position.setY( bone_frame_dic["センター"][n].position.y() - ankle_pos_max )
            
            # logger.debug("center.y2:{0}".format(bone_frame_dic["センター"][n].position.y()))    

            # X回転もさせず、接地させる
            left_ik_rotation.setX(0)
            right_ik_rotation.setX(0)

        # ジャンプしてる時と浮いてる時の区別がつかないので、一旦保留        
        # if bone_frame_dic["センター"][n].position.y() > 0 \
        #     and left_ankle_pos.y() >= 0 and abs(left_ik_rotation.toEulerAngles().x()) < 20 and abs(left_ik_rotation.toEulerAngles().y()) < 20 and abs(left_ik_rotation.toEulerAngles().z()) < 20 \
        #     and right_ankle_pos.y() >= 0 and abs(right_ik_rotation.toEulerAngles().x()) < 20 and abs(right_ik_rotation.toEulerAngles().y()) < 20 and abs(right_ik_rotation.toEulerAngles().z()) < 20:
        #     # Y軸が浮いていて、かつ足の角度が小さい場合、下向きに補正

        #     # センターを補正
        #     new_center_y = 0 - ( ( left_ankle_pos.y() + right_ankle_pos.y() ) / 2 ) 
        #     # しゃがんでる場合、もともとセンターがマイナスの可能性があるので、その場合は上書きしない
        #     if bone_frame_dic["センター"][n].position.y() > new_center_y:
        #         logger.debug("浮きセンターY上書き n={0}, y={1}, new_y={2}".format(n, bone_frame_dic["センター"][n].position.y(), new_center_y))
        #         bone_frame_dic["センター"][n].position.setY(new_center_y)

        #     # Y軸はセンターマイナスで接地させる
        #     left_ankle_pos.setY(0)
        #     right_ankle_pos.setY(0)

        #     # X回転もさせず、接地させる
        #     left_ik_rotation.setX(0)
        #     right_ik_rotation.setX(0)

        if left_ankle_pos.y() < 0 and right_ankle_pos.y() >= 0:
            # センターが少ししゃがんでて、足が浮いている場合、下ろす
            # 左足だけの場合マイナス値は0に補正
            left_ankle_pos.setY(0)

            # X回転もさせず、接地させる
            left_ik_rotation.setX(0)

        if (right_ankle_pos.y() < 0 and left_ankle_pos.y() >= 0):
            # 右足だけの場合マイナス値は0に補正
            right_ankle_pos.setY(0)

            # X回転もさせず、接地させる
            right_ik_rotation.setX(0)

        # if abs(bone_frame_dic["上半身"][n].rotation.toEulerAngles().y()) < 30 and left_ankle_pos.y() == 0:
        #     # 正面向きでY位置が0の場合、回転させず、接地させる
        #     left_ik_rotation.setX(0)
        #     left_ik_rotation.setY(0)
        #     left_ik_rotation.setZ(0)

        # if abs(bone_frame_dic["上半身"][n].rotation.toEulerAngles().y()) < 30 and right_ankle_pos.y() == 0:
        #     # 正面向きでY位置が0の場合、回転させず、接地させる
        #     right_ik_rotation.setX(0)
        #     right_ik_rotation.setY(0)
        #     right_ik_rotation.setZ(0)

        # センターと足首までのY距離
        bone_center_ankle_y = center_bone[1] - right_ankle_bone[1]
        # logger.debug("bone_center_leg_y {0}".format(bone_center_leg_y))

        # 足IKの位置が0で、センターが沈んでいる場合、ボーンのセンター位置に合わせて少しずらす
        if ( abs(bone_frame_dic["センター"][n].position.y()) > bone_center_ankle_y ):
            new_center_y = bone_frame_dic["センター"][n].position.y() - ( center_bone[1] - right_leg_bone[1] )
            logger.debug("陥没センターY上書き n={0}, y={1}, new_y={2}".format(n, bone_frame_dic["センター"][n].position.y(), new_center_y))
            bone_frame_dic["センター"][n].position.setY(new_center_y)

        bone_frame_dic["左足ＩＫ"][n].position = left_ankle_pos
        bone_frame_dic["左足ＩＫ"][n].rotation = left_ik_rotation
        bone_frame_dic["左足"][n].rotation = left_leg_diff_rotation

        bone_frame_dic["右足ＩＫ"][n].position = right_ankle_pos
        bone_frame_dic["右足ＩＫ"][n].rotation = right_ik_rotation
        bone_frame_dic["右足"][n].rotation = right_leg_diff_rotation

        # if n >= 1800:
        #     sys.exit()

    #　ひざは登録除去
    bone_frame_dic["左ひざ"] = []
    bone_frame_dic["右ひざ"] = []



# 行列でIKの位置を求める
def calc_IK_matrix(center_bone, lower_body_bone, leg_bone, knee_bone, ankle_bone, toes_bone, ik_bone, center_pos, lower_body_rotation, leg_rotation, knee_rotation):

    # logger.debug("calc_IK_matrix ------------------------")

    # IKを求める ----------------------------

    # ローカル位置
    trans_vs = [0 for i in range(6)]
    # センターのローカル位置
    trans_vs[0] = center_bone + center_pos - ik_bone
    # 下半身のローカル位置
    trans_vs[1] = lower_body_bone - center_bone
    # 足のローカル位置
    trans_vs[2] = leg_bone - lower_body_bone
    # ひざのローカル位置 
    trans_vs[3] = knee_bone - leg_bone
    # 足首のローカル位置
    trans_vs[4] = ankle_bone - knee_bone
    # つま先のローカル位置
    trans_vs[5] = toes_bone - ankle_bone
    
    # 加算用クォータニオン
    add_qs = [0 for i in range(6)]
    # センターの回転
    add_qs[0] = QQuaternion()
    # 下半身の回転
    add_qs[1] = lower_body_rotation
    # 足の回転
    add_qs[2] = leg_rotation
    # ひざの回転
    add_qs[3] = knee_rotation
    # 足首の回転
    add_qs[4] = QQuaternion()
    # つま先の回転
    add_qs[5] = QQuaternion()

    # 行列
    matrixs = [0 for i in range(6)]

    for n in range(len(matrixs)):
        # 行列を生成
        matrixs[n] = QMatrix4x4()
        # 移動
        matrixs[n].translate(trans_vs[n])
        # 回転
        matrixs[n].rotate(add_qs[n])

        # logger.debug("matrixs[n] n={0}".format(n))
        # logger.debug(matrixs[n])

    # 足付け根の位置
    leg_pos = matrixs[0] * matrixs[1] * QVector4D(trans_vs[2], 1)

    # logger.debug("leg_pos")
    # logger.debug(leg_pos.toVector3D())

    # ひざの位置
    knee_pos = matrixs[0] * matrixs[1] * matrixs[2] * QVector4D(trans_vs[3], 1)

    # logger.debug("knee_pos")
    # logger.debug(knee_pos.toVector3D())

    # 足首の位置(行列の最後は掛けない)
    ankle_pos = matrixs[0] * matrixs[1] * matrixs[2] * matrixs[3] * QVector4D(trans_vs[4], 1)
    # logger.debug("ankle_pos {0}".format(ankle_pos.toVector3D()))

    # logger.debug("ankle_pos")
    # logger.debug(ankle_pos.toVector3D())

    # つま先の位置
    toes_pos = matrixs[0] * matrixs[1] * matrixs[2] * matrixs[3] * matrixs[4] * QVector4D(trans_vs[5], 1)
    # logger.debug("toes_pos {0}".format(toes_pos.toVector3D()))

    # 足付け根から足首までの距離
    ankle_leg_diff = ankle_pos - leg_pos

    # logger.debug("ankle_leg_diff")
    # logger.debug(ankle_leg_diff)
    # logger.debug(ankle_leg_diff.length())

    # ひざから足付け根までの距離
    knee_leg_diff = knee_bone - leg_bone

    # logger.debug("knee_leg_diff")
    # logger.debug(knee_leg_diff)
    # logger.debug(knee_leg_diff.length())

    # 足首からひざまでの距離
    ankle_knee_diff = ankle_bone - knee_bone

    # logger.debug("ankle_knee_diff")
    # logger.debug(ankle_knee_diff)
    # logger.debug(ankle_knee_diff.length())

    # つま先から足首までの距離
    toes_ankle_diff = toes_bone - ankle_bone

    # logger.debug("toes_ankle_diff")
    # logger.debug(toes_ankle_diff)
    # logger.debug(toes_ankle_diff.length())

    # 三辺から角度を求める

    # 足の角度
    leg_angle = calc_leg_angle(ankle_leg_diff, knee_leg_diff, ankle_knee_diff)
    # logger.debug("leg_angle:   {0}".format(leg_angle))

    # ひざの角度
    knee_angle = calc_leg_angle(knee_leg_diff, ankle_knee_diff, ankle_leg_diff)
    # logger.debug("knee_angle:  {0}".format(knee_angle))

    # 足首の角度
    ankle_angle = calc_leg_angle(ankle_knee_diff, ankle_leg_diff, knee_leg_diff)
    # logger.debug("ankle_angle: {0}".format(ankle_angle))

    # 足の回転 ------------------------------

    # 足の付け根からひざへの方向を表す青い単位ベクトル(長さ1)
    # 足の付け根から足首へのベクトルをX軸回りに回転させる
    knee_v = QQuaternion.fromEulerAngles(leg_angle * -1, 0, 0) * ankle_leg_diff.toVector3D().normalized()

    # logger.debug("knee_v")
    # logger.debug(knee_v)

    # FKのひざの位置
    ik_knee_3d = knee_v * knee_leg_diff.length() + leg_pos.toVector3D()

    # logger.debug("ik_knee_3d")
    # logger.debug(ik_knee_3d)

    # IKのひざ位置からFKのひざ位置に回転させる
    leg_diff_rotation = QQuaternion.rotationTo(knee_pos.toVector3D(), ik_knee_3d)

    # logger.debug("leg_diff_rotation")
    # logger.debug(leg_diff_rotation)
    # logger.debug(leg_diff_rotation.toEulerAngles())

    # 足IKの回転（足首の角度）-------------------------

    # FKと同じ状態の足首の向き
    ik_rotation = lower_body_rotation * leg_rotation * knee_rotation
    # logger.debug("ik_rotation {0}".format(ik_rotation.toEulerAngles()))

    return (ankle_pos.toVector3D(), ik_rotation, leg_diff_rotation)



# 三辺から足の角度を求める
def calc_leg_angle(a, b, c):

    cos = ( pow(a.length(), 2) + pow(b.length(), 2) - pow(c.length(), 2) ) / ( 2 * a.length() * b.length() )

    # logger.debug("cos")
    # logger.debug(cos)

    radian = np.arccos(cos)

    # logger.debug("radian")
    # logger.debug(radian)

    angle = np.rad2deg(radian)

    # logger.debug("angle")
    # logger.debug(angle)

    return angle

# センターZの計算 
def calc_center_z(smoothed_2d, depths, start_frame, upright_idxs, center_xy_scale, center_z_scale, target_upright_idx, target_start_pos):

    if center_z_scale == 0:
        return

    # 直立インデックス 
    upright_idx = upright_idxs[0]

    # 全フレームの推定深度
    depth_all_frames = []

    # 添え字と腰深度の配列
    depth_indexes = np.array(depths)[:, DEPTH_INDEX["index"]:DEPTH_INDEX["index"]+1].flatten()
    waist_depth_values = np.array(depths)[:, DEPTH_INDEX["Wrist"]:DEPTH_INDEX["Wrist"]+1].flatten()

    # logger.info(depth_indexes)
    # logger.info(waist_depth_values)

    # もっともカメラに近付いた距離
    nearest_depth = np.min(waist_depth_values)

    # もっともカメラから遠のいた距離
    furthest_depth = np.max(waist_depth_values)

    # カメラからの遠近差
    perspective_diff = furthest_depth - nearest_depth

    logger.debug("perspective_diff: {0}".format(perspective_diff))

    # 遠近差をZスケールで割る
    # 深度に対するZ移動量
    perspective_diff_zscale = center_z_scale / perspective_diff

    logger.debug("perspective_diff_zscale: {0}".format(perspective_diff_zscale))

    # 直立にもっとも近いINDEXを取得する
    upright_nearest_idx = get_nearest_idx(depth_indexes, upright_idx)

    logger.debug("upright_nearest_idx: {0}".format(upright_nearest_idx))

    # 直立近似INDEXの深度を取得する
    upright_depth = waist_depth_values[upright_nearest_idx]

    logger.debug("upright_depth: {0}".format(upright_depth))

    # 直立INDEXの上半身の長さ
    upright_upper_length = (((smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["RHip"]].y() + smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["LHip"]].y() ) / 2) - smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["Neck"]].y()) / center_xy_scale    

    tu_nidxs = []
    tu_upper_lengths = []
    tu_depths = []
    for tui in upright_idxs:

        # 直立にもっとも近いINDEXを取得する
        tunidx = int(get_nearest_idx(depth_indexes, tui))

        # logger.info(tui)
        # logger.info(tunidx)

        # 体幹直立近似フレームの深度インデックス
        d_tunidx = int(depth_indexes[tunidx])

        # 既に同じインデックスがあったらスルー
        if d_tunidx in tu_nidxs:
            continue

        # 体幹直立近似フレームのインデックス
        tu_nidxs.append(d_tunidx)
        # 体幹直立近似フレームの上半身の長さ
        tu_upper_lengths.append( ((( smoothed_2d[d_tunidx][SMOOTHED_2D_INDEX["RHip"]].y() + smoothed_2d[d_tunidx][SMOOTHED_2D_INDEX["LHip"]].y() ) / 2) - smoothed_2d[d_tunidx][SMOOTHED_2D_INDEX["Neck"]].y()) / center_xy_scale )
        # 体幹直立近似フレームの深度
        tu_depths.append(float(waist_depth_values[tunidx]) * center_z_scale)

        # 3件超えたら終了  
        if len(tu_nidxs) >= 3:
            break
        
    logger.debug(tu_nidxs)
    logger.debug(tu_upper_lengths)
    logger.debug(tu_depths)
    
    # 視野角
    radians = []
    for n in range(len(tu_nidxs[:3])):
        radian = math.atan2( tu_upper_lengths[n], tu_depths[n] )
        deg = math.degrees(radian)
        logger.debug("x={0}, y={1}, r={2}, d={3}".format(tu_depths[n], tu_upper_lengths[n], radian, deg))
        
        radians.append(radian)

    view_radian = np.average(np.array(radians))
    logger.info("画角推定 ラジアン=%s 角度=%s", view_radian, math.degrees(view_radian))

    # 直立だった場合の上半身の長さ
    upright_upper_length_estimate = upright_depth * math.tan(view_radian)

    for idx, n in enumerate(depth_indexes) :
        # 深度のINDEX1件ごとにセンターZ計算
        nn = int(n)

        # 開始フレームインデックスまでは飛ばす
        if nn <= start_frame:
            continue

        # 現在の深度
        now_depth = waist_depth_values[idx] - upright_depth

        # センターZ
        center_z = now_depth * center_z_scale * perspective_diff_zscale

        # 直立だった場合の上半身の長さ
        now_upright_upper_length = (now_depth + upright_depth) * math.tan(view_radian)

        # センターY調整値
        center_y_adjust = (upright_upper_length_estimate - now_upright_upper_length) / center_z_scale

        logger.debug("nn: {0}, d: {1}, z: {2}, l:{3} y: {4}".format(nn, now_depth, center_z, now_upright_upper_length, center_y_adjust))
        
        # センターZ
        bone_frame_dic["センター"][nn - start_frame].position.setZ(float(center_z) + target_start_pos["center"].z())
        
        # センターY
        bone_frame_dic["センター"][nn - start_frame].position.setY(bone_frame_dic["センター"][nn - start_frame].position.y() + float(center_y_adjust))

        # 深度リストに追加
        depth_all_frames.append(float(now_depth))

        if nn > 0:
            # 1F以降の場合、その間のセンターも埋める
            prev_depth = waist_depth_values[idx - 1] - upright_depth
            prev_idx = int(depth_indexes[idx - 1])

            # 前回との間隔
            interval = nn - prev_idx

            # 前回との間隔の差分
            diff_depth = now_depth - prev_depth

            logger.debug("prev_idx: {0}, prev_depth: {1}, interval: {2}, diff_depth: {3}".format(prev_idx, prev_depth, interval, diff_depth))
            
            for midx, m in enumerate(range(prev_idx + 1, nn)):
                interval_depth = prev_depth + ( (diff_depth / interval) * (midx + 1) )

                # 深度リストに追加
                depth_all_frames.append(float(interval_depth))

                # センターZ
                center_z = interval_depth * center_z_scale * perspective_diff_zscale

                # 直立だった場合の上半身の長さ
                now_upright_upper_length = (interval_depth + upright_depth) * math.tan(view_radian)

                # センターY調整値
                center_y_adjust = (upright_upper_length_estimate - now_upright_upper_length) / center_z_scale

                logger.debug("m: {0}, d: {1}, z: {2}, l:{3} y: {4}".format(m, interval_depth, center_z, now_upright_upper_length, center_y_adjust))

                bone_frame_dic["センター"][m - start_frame].position.setZ(float(center_z) + target_start_pos["center"].z())
                
                # センターY
                bone_frame_dic["センター"][m - start_frame].position.setY(bone_frame_dic["センター"][m - start_frame].position.y() + float(center_y_adjust))

    return depth_all_frames
                
def get_nearest_idx(target_list, num):
    """
    概要: リストからある値に最も近い値のINDEXを返却する関数
    @param target_list: データ配列
    @param num: 対象値
    @return 対象値に最も近い値のINDEX
    """

    # logger.debug(target_list)
    # logger.debug(num)

    # リスト要素と対象値の差分を計算し最小値のインデックスを取得
    idx = np.abs(np.asarray(target_list) - num).argmin()
    return idx

# センターの計算
def calc_center(smoothed_2d, bone_csv_file, upright_idxs, center_xy_scale, center_z_scale, heelpos, target_upright_idx, target_start_pos):

    if center_xy_scale == 0:
        return

    logger.debug("bone_csv_file: "+ bone_csv_file)

    # 直立インデックス
    upright_idx = upright_idxs[0]    

    # ボーンファイルを開く
    with open(bone_csv_file, "r",  encoding=get_file_encoding(bone_csv_file)) as bf:
        reader = csv.reader(bf)

        for row in reader:

            if row[1] == "センター" or row[2].lower() == "center":
                # センターボーン
                center_3d = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "首" or row[2].lower() == "neck":
                # 首ボーン
                neck_3d = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右足" or row[2].lower() == "leg_r":
                # 右足ボーン
                right_leg_3d = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左足" or row[2].lower() == "leg_l":
                # 左足ボーン
                left_leg_3d = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "右足首" or row[2].lower() == "ankle_r":
                # 右足首ボーン
                right_ankle_3d = QVector3D(float(row[5]), float(row[6]), float(row[7]))

            if row[1] == "左足首" or row[2].lower() == "ankle_l":
                # 左足首ボーン
                left_ankle_3d = QVector3D(float(row[5]), float(row[6]), float(row[7]))

    # logger.debug("neck_3d")
    # logger.debug(neck_3d)
    # logger.debug("right_leg_3d")
    # logger.debug(right_leg_3d)
    # logger.debug("left_leg_3d")
    # logger.debug(left_leg_3d)
    # logger.debug("center_3d")
    # logger.debug(center_3d)

    # ボーン頂点からの三角形面積
    bone_upright_area = calc_triangle_area(neck_3d, right_leg_3d, left_leg_3d)

    # logger.debug("smoothed_2d[upright_idx]")
    # logger.debug(smoothed_2d[upright_idx])

    # 直立フレームの三角形面積
    smoothed_upright_area = calc_triangle_area(smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["Neck"]], smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["RHip"]], smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["LHip"]])

    # logger.debug("upright_area")
    # logger.debug(smoothed_upright_area)

    # ボーンと映像の三角形比率(スケール調整あり)
    upright_xy_scale = bone_upright_area / smoothed_upright_area * center_xy_scale

    # logger.debug("upright_scale")
    # logger.debug(upright_xy_scale)

    # 直立フレームの左足と右足の位置の平均
    upright_leg_avg = abs((smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["RHip"]].y() + smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["LHip"]].y()) / 2)

    # 直立フレームの左足首と右足首の位置の平均
    upright_ankle_avg = abs((smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["RAnkle"]].y() + smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["LAnkle"]].y()) / 2)

    # logger.debug("upright_ankle_avg")
    # logger.debug(upright_ankle_avg)

    # ボーンの足首のY位置
    bone_anke_y = (right_ankle_3d[1] + left_ankle_3d[1]) / 2

    # logger.debug("bone_anke_y")
    # logger.debug(bone_anke_y)

    # 足首位置の比率(上半身のみでゼロ割対策)
    if upright_ankle_avg != 0:
        upright_ankle_scale = (bone_anke_y / upright_ankle_avg) * (center_xy_scale / 100)
    else:
        upright_ankle_scale = 0

    # # 上半身から首までの距離
    # neck_upright_distance = upper_body_3d.distanceToPoint(neck_3d)

    # # logger.debug("neck_upright_distance")
    # # logger.debug(neck_upright_distance)

    # # 上半身から左足までの距離
    # left_leg_upright_distance = upper_body_3d.distanceToPoint(left_leg_3d)

    # # logger.debug("left_leg_upright_distance")
    # # logger.debug(left_leg_upright_distance)

    # # 上半身から左足までの距離
    # right_leg_upright_distance = upper_body_3d.distanceToPoint(right_leg_3d)
    
    # logger.debug("right_leg_upright_distance")
    # logger.debug(right_leg_upright_distance)

    # 3Dでの首・左足・右足の投影三角形
    # pos_upright_area = calc_triangle_area(positions_multi[upright_idx][8], positions_multi[upright_idx][1], positions_multi[upright_idx][4])

    for n, smoothed in enumerate(smoothed_2d):
        logger.debug("センター計算 frame={0}".format(n))

        # 左足と右足の位置の小さい方
        ankle_min = np.min([smoothed_2d[n][SMOOTHED_2D_INDEX["RAnkle"]].y(), smoothed_2d[n][SMOOTHED_2D_INDEX["LAnkle"]].y()])

        # logger.debug("ankle_min")
        # logger.debug(ankle_min)

        # logger.debug("ankle_min * upright_ankle_scale")
        # logger.debug(ankle_min * upright_ankle_scale)

        # 左足と右足の位置の平均
        leg_avg = abs((smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]].y() + smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]].y()) / 2)
        
        # 足の上下差
        leg_diff = upright_leg_avg - leg_avg

        # Y軸移動(とりあえずセンター固定)
        center_y = (leg_diff * upright_xy_scale) - (ankle_min * upright_ankle_scale)

        # 踵補正を入れて設定する
        bone_frame_dic["センター"][n].position.setY(center_y + heelpos)
        # bone_frame_dic["センター"][n].position.setY((leg_diff * upright_xy_scale))
        
        # 首・左足・右足の中心部分をX軸移動
        x_avg = ((smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]].x() + smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]].x() + smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]].x()) / 3) \
                    - smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["Neck"]].x()
        center_x = x_avg * upright_xy_scale
        bone_frame_dic["センター"][n].position.setX(center_x + target_start_pos["center"].x())

        logger.debug("center {0} x={1}, y={2}".format(n, center_x, center_y))

        # 現在の映像の三角形面積
        # now_smoothed_area = calc_triangle_area(smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]], smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]], smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]])

        # logger.debug("smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]]")
        # logger.debug(smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]])

        # logger.debug("smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]]")
        # logger.debug(smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]])

        # logger.debug("smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]]")
        # logger.debug(smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]])

        # logger.debug("now_smoothed_area")
        # logger.debug(now_smoothed_area)

        # # 首の位置を上半身の傾きから求める
        # upper_slope = QQuaternion(0, 0, -1, 0).inverted() * bone_frame_dic["上半身"][n].rotation.normalized() * neck_upright_distance

        # # 左足の位置を下半身の傾きから求める
        # left_leg_slope = QQuaternion(0, 0.2, 1, 0).inverted() * bone_frame_dic["下半身"][n].rotation.normalized() * left_leg_upright_distance

        # # 右足の位置を下半身の傾きから求める
        # right_leg_slope = QQuaternion(0, -0.2, 1, 0).inverted() * bone_frame_dic["下半身"][n].rotation.normalized() * right_leg_upright_distance

        # # 現在のボーン構造の三角形面積
        # now_bone_area = calc_triangle_area(upper_slope.vector(), left_leg_slope.vector(), right_leg_slope.vector())
        
        # logger.debug("smoothed_upright_area")
        # logger.debug(smoothed_upright_area)

        # 3Dでの首・左足・右足の投影三角形
        # pos_now_area = calc_triangle_area(positions_multi[n][8], positions_multi[n][1], positions_multi[n][4])

        # logger.debug("positions_multi[n][8]")
        # logger.debug(positions_multi[n][8])
        # logger.debug("positions_multi[n][1]")
        # logger.debug(positions_multi[n][1])
        # logger.debug("positions_multi[n][4]")
        # logger.debug(positions_multi[n][4])

        # logger.debug("pos_upright_area")
        # logger.debug(pos_upright_area)

        # logger.debug("pos_now_area")
        # logger.debug(pos_now_area)

        # 3Dでの現在の縮尺
        # pos_scale = pos_now_area / pos_upright_area

        # logger.debug("pos_scale")
        # logger.debug(pos_scale)

        # logger.debug("pos_scale ** 2")
        # logger.debug(pos_scale ** 2)

        # 2Dでの首・左足・右足の投影三角形
        # smoothed_now_area = calc_triangle_area(smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]], smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]], smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]])

        # logger.debug("smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]]")    
        # logger.debug(smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]])
        # logger.debug("smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]]")    
        # logger.debug(smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]])
        # logger.debug("smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]]")    
        # logger.debug(smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]])

        # logger.debug("smoothed_upright_area")
        # logger.debug(smoothed_upright_area)

        # logger.debug("smoothed_now_area")
        # logger.debug(smoothed_now_area)

        # 2Dでの現在の縮尺
        # smoothed_scale = smoothed_now_area / smoothed_upright_area

        # logger.debug("smoothed_scale")
        # logger.debug(smoothed_scale)

        # logger.debug("((1 - smoothed_scale) ** 2)")
        # logger.debug(((1 - smoothed_scale) ** 2))

        # Z軸移動位置の算出
        # now_z_scale = pos_scale * (1 - smoothed_scale)

        # logger.debug("now_z_scale")
        # logger.debug(now_z_scale)

        # logger.debug("now_z_scale * center_z_scale")
        # logger.debug(now_z_scale * center_z_scale)

        # Z軸の移動補正
        # bone_frame_dic["センター"][n].position.setZ(now_z_scale * center_z_scale)


        # # 上半身の各軸傾き具合
        # rx = bone_frame_dic["上半身"][n].rotation.toEulerAngles().x()
        # ry = bone_frame_dic["上半身"][n].rotation.toEulerAngles().y() * -1
        # rz = bone_frame_dic["上半身"][n].rotation.toEulerAngles().z() * -1

        # # 傾いたところの頂点：首（傾きを反転させて正面向いた形にする）
        # smoothed_upright_slope_neck = calc_slope_point(smoothed_2d[upright_idx][8], rx * -1, ry * -1, rz * -1)
        # # 傾いたところの頂点：左足
        # smoothed_upright_slope_left_leg = calc_slope_point(smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["RHip"]], rx * -1, ry * -1, rz * -1)
        # # 傾いたところの頂点：右足
        # smoothed_upright_slope_right_leg = calc_slope_point(smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["LHip"]], rx * -1, ry * -1, rz * -1)

        # # 傾きを反転させた直立面積
        # smoothed_upright_slope_area = calc_triangle_area(smoothed_upright_slope_neck, smoothed_upright_slope_left_leg, smoothed_upright_slope_right_leg)

        # logger.debug("smoothed_upright_slope_area")
        # logger.debug(smoothed_upright_slope_area)

        # logger.debug("smoothed_upright_area")
        # logger.debug(smoothed_upright_area)

        # # 直立の関節の回転分面積を現在の関節面積で割って、大きさの比率を出す
        # now_z_scale = smoothed_upright_slope_area / smoothed_upright_area

        # if n == 340 or n == 341:

        #     logger.debug("smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["Neck"]]")
        #     logger.debug(smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["Neck"]])

        #     logger.debug("smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["RHip"]]")
        #     logger.debug(smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["RHip"]])

        #     logger.debug("smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["LHip"]]")
        #     logger.debug(smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["LHip"]])

        #     logger.debug("smoothed_upright_slope_neck")
        #     logger.debug(smoothed_upright_slope_neck)

        #     logger.debug("smoothed_upright_slope_left_leg")
        #     logger.debug(smoothed_upright_slope_left_leg)

        #     logger.debug("smoothed_upright_slope_right_leg")
        #     logger.debug(smoothed_upright_slope_right_leg)

        #     logger.debug("smoothed_upright_slope_area")
        #     logger.debug(smoothed_upright_slope_area)

        # # 傾きの総数 - 各傾きの絶対値＝傾き具合
        # rsum = (90 - abs(rx)) + (90 - abs(90 - abs(ry))) + (90 - abs(rz))
        # # 360で割って、どれくらい傾いているか係数算出(1に近いほど正面向き)
        # rsum_scale = (180 / rsum) ** ( center_z_scale ** center_z_scale)

        # logger.debug("rx")
        # logger.debug(rx)

        # logger.debug("ry")
        # logger.debug(ry)

        # logger.debug("rz")
        # logger.debug(rz)

        # logger.debug("rsum")
        # logger.debug(rsum)

        # logger.debug("rsum_scale")
        # logger.debug(rsum_scale)

        # logger.debug("now_z_scale")
        # logger.debug(now_z_scale)
            
        # # 1より大きい場合、近くにある(マイナス)
        # # 1より小さい場合、遠くにある(プラス)
        # now_z_scale_pm = 1 - now_z_scale

        # logger.debug("now_z_scale_pm")
        # logger.debug(now_z_scale_pm)

        # logger.debug("now_z_scale_pm * rsum_scale")
        # logger.debug(now_z_scale_pm * rsum_scale)

        # if n < 20:
        # logger.debug("upper_slope")
        # logger.debug(upper_slope)
        # logger.debug(upper_slope.vector())

        # logger.debug("left_leg_slope")
        # logger.debug(left_leg_slope)
        # logger.debug(left_leg_slope.vector())

        # logger.debug("right_leg_slope")
        # logger.debug(right_leg_slope)
        # logger.debug(right_leg_slope.vector())

        # logger.debug("bone_upright_area")
        # logger.debug(bone_upright_area)

        # logger.debug("now_bone_area")
        # logger.debug(now_bone_area)

        # logger.debug("now_scale")
        # logger.debug(now_scale)

        # logger.debug("now_scale * center_scale")
        # logger.debug(now_scale * center_scale)

        # logger.debug("leg_avg")
        # logger.debug(leg_avg)
        
        # logger.debug("leg_diff")
        # logger.debug(leg_diff)
        
        # logger.debug("smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["Neck"]].x()")
        # logger.debug(smoothed_2d[upright_idx][SMOOTHED_2D_INDEX["Neck"]].x())
        
        # logger.debug("smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]].x()")
        # logger.debug(smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]].x())
        
        # logger.debug("smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]].x()")
        # logger.debug(smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]].x())
        
        # logger.debug("smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]].x()")
        # logger.debug(smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]].x())
        
        # logger.debug("((smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]].x() + smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]].x() + smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]].x()) / 3)")
        # logger.debug(((smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]].x() + smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]].x() + smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]].x()) / 3))
        
        # logger.debug("x_avg")
        # logger.debug(x_avg)

        # logger.debug("now_smoothed_area")
        # logger.debug(now_smoothed_area)

        # logger.debug("now_position_area")
        # logger.debug(now_position_area)

        # logger.debug("now_scale")
        # logger.debug(now_scale)

        # logger.debug("now_scale * upright_position_scale")
        # logger.debug(now_scale * upright_position_scale)

        

        # # モデルの上半身の傾き
        # upper_body_euler = QVector3D(
        #     bone_frame_dic["上半身"][n].rotation.toEulerAngles().x() \
        #     , bone_frame_dic["上半身"][n].rotation.toEulerAngles().y() * -1 \
        #     , bone_frame_dic["上半身"][n].rotation.toEulerAngles().z() * -1 \
        # ) 



        # # モデルの上半身の傾き。初期位置からフレームの位置まで回転
        # upper_body_qq = QQuaternion(0, upper_body_3d)
        # # upper_body_qq = QQuaternion.rotationTo( upper_body_3d, bone_frame_dic["上半身"][n].rotation.vector() )

        # if n < 20:
        #     logger.debug("bone_frame_dic")
        #     logger.debug(bone_frame_dic["上半身"][n].rotation)
        #     logger.debug(bone_frame_dic["上半身"][n].rotation.toVector4D())
        #     logger.debug(bone_frame_dic["上半身"][n].rotation.toEulerAngles())
        #     v = bone_frame_dic["上半身"][n].rotation.toVector4D()
        #     logger.debug(v.x() * v.w())
        #     logger.debug(v.y() * v.w() * -1)
        #     logger.debug(v.z() * v.w() * -1)
        #     logger.debug("upper_body_qq")
        #     logger.debug(upper_body_qq)
        #     logger.debug(upper_body_qq.toEulerAngles())
        #     logger.debug(upper_body_qq.toVector4D())

DEPTH_INDEX = {
    "index": 0,
    "Wrist": 1,
    "RAnkle": 2,
    "LAnkle": 3
}

# depthファイルの読み込み
def load_depth(depth_file):
    if os.path.exists(depth_file) == False:
        return None

    depths = [[0 for i in range(4)] for j in range(sum(1 for line in open(depth_file)))]

    n = 0
    # 深度ファイルからフレームINDEXを取得する
    with open(depth_file, "r") as bf:
        # カンマ区切りなので、csvとして読み込む
        reader = csv.reader(bf)

        for row in reader:
            logger.debug("row[0] {0}, row[1]: {1}, row[2]: {2}, row[3]: {3}".format(row[0], row[1], row[2], row[3]))
            depths[n][DEPTH_INDEX["index"]] = int(row[0])
            depths[n][DEPTH_INDEX["Wrist"]] = float(row[1])
            depths[n][DEPTH_INDEX["RAnkle"]] = float(row[2])
            depths[n][DEPTH_INDEX["LAnkle"]] = float(row[3])
        
            n += 1

    return depths

SMOOTHED_2D_INDEX = {
    "Nose": 0,
    "Neck": 1,
    "RShoulder": 2,
    "RElbow": 3,
    "RWrist": 4,
    "LShoulder": 5,
    "LElbow": 6,
    "LWrist": 7,
    "RHip": 8,
    "RKnee": 9,
    "RAnkle": 10,
    "LHip": 11,
    "LKnee": 12,
    "LAnkle": 13,
    "REye": 14,
    "LEye": 15,
    "REar": 16,
    "LEar": 17,
    "Background": 18
}

# 関節2次元情報を取得
def load_smoothed_2d(smoothed_file):
    # １次元：フレーム数分
    # ２次元：OpenposeのINDEX分
    smoothed_2d = [[0 for i in range(19)] for j in range(sum(1 for line in open(smoothed_file)))]
    n = 0
    with open(smoothed_file, "r") as sf:
        line = sf.readline() # 1行を文字列として読み込む(改行文字も含まれる)
        
        while line:
            # 空白で複数項目に分解
            smoothed = re.split("\s+", line)

            # logger.debug(smoothed)

            # 首の位置
            smoothed_2d[n][SMOOTHED_2D_INDEX["Neck"]] = QVector3D(float(smoothed[2]), float(smoothed[3]), 0)
            # 右足付け根
            smoothed_2d[n][SMOOTHED_2D_INDEX["RHip"]] = QVector3D(float(smoothed[16]), float(smoothed[17]), 0)
            # 左足付け根
            smoothed_2d[n][SMOOTHED_2D_INDEX["LHip"]] = QVector3D(float(smoothed[22]), float(smoothed[23]), 0)
            # 右ひざ
            smoothed_2d[n][SMOOTHED_2D_INDEX["RKnee"]] = QVector3D(float(smoothed[18]), float(smoothed[19]), 0)
            # 左ひざ
            smoothed_2d[n][SMOOTHED_2D_INDEX["LKnee"]] = QVector3D(float(smoothed[24]), float(smoothed[25]), 0)
            # 右足首
            smoothed_2d[n][SMOOTHED_2D_INDEX["RAnkle"]] = QVector3D(float(smoothed[20]), float(smoothed[21]), 0)
            # 左足首
            smoothed_2d[n][SMOOTHED_2D_INDEX["LAnkle"]] = QVector3D(float(smoothed[26]), float(smoothed[27]), 0)
        
            n += 1

            line = sf.readline()
    
    return smoothed_2d

# センターY軸をグルーブY軸に移管
def set_groove(bone_csv_file):

    # グルーブボーンがあるか
    is_groove = False
    # ボーンファイルを開く
    with open(bone_csv_file, "r", encoding=get_file_encoding(bone_csv_file)) as bf:
        reader = csv.reader(bf)

        for row in reader:
            if row[1] == "グルーブ" or row[2].lower() == "groove":
                is_groove = True
                break

    if is_groove:

        for n in range(len(bone_frame_dic["センター"])):
            # logger.debug("グルーブ移管 frame={0}".format(n))

            # グルーブがある場合、Y軸をグルーブに設定
            bone_frame_dic["グルーブ"][n].position = QVector3D(0, bone_frame_dic["センター"][n].position.y(), 0)
            bone_frame_dic["センター"][n].position = QVector3D(bone_frame_dic["センター"][n].position.x(), 0, bone_frame_dic["センター"][n].position.z())

    return is_groove

# 上半身2があるか    
def is_upper2_body_bone(bone_csv_file):

    # ボーンファイルを開く
    with open(bone_csv_file, "r", encoding=get_file_encoding(bone_csv_file)) as bf:
        reader = csv.reader(bf)

        for row in reader:
            if row[1] == "上半身2" or row[2].lower() == "upper body2":
                return True
    
    return False

# 直立姿勢から傾いたところの頂点を求める
# FIXME クォータニオンで求められないか要調査
def calc_slope_point(upright, rx, ry, rz):
    # // ｘ軸回転
    # x1 = dat[n][0] ;
    # y1 = dat[n][1]*cos(rx)-dat[n][2]*sin(rx) ;
    # z1 = dat[n][1]*sin(rx)+dat[n][2]*cos(rx) ;
    x1 = upright.x()
    y1 = upright.y() * np.cos(np.radians(rx)) - upright.z() * np.sin(np.radians(rx))
    z1 = upright.y() * np.sin(np.radians(rx)) + upright.z() * np.cos(np.radians(rx))

    # // ｙ軸回転
    # x2 = x1*cos(ry)+z1*sin(ry) ;
    # y2 = y1 ;
    # z2 = z1*cos(ry)-x1*sin(ry) ;
    x2 = x1 * np.cos(np.radians(ry)) + z1 * np.sin(np.radians(ry))
    y2 = y1
    z2 = z1 * np.cos(np.radians(ry)) - x1 * np.sin(np.radians(ry))

    # // ｚ軸回転
    # x3 = x2*cos(rz)-y2*sin(rz) ;
    # y3 = x2*sin(rz)+y2*cos(rz) ;
    # z3 = z2 ;
    x3 = x2 * np.cos(np.radians(rz)) - y2 * np.sin(np.radians(rz))
    y3 = x2 * np.sin(np.radians(rz)) + y2 * np.cos(np.radians(rz))
    z3 = z2

    return QVector3D(x3, y3, z3)

# 3つの頂点から三角形の面積を計算する
def calc_triangle_area(a, b, c):
    # logger.debug(a)
    # logger.debug(b)
    # logger.debug(c)
    # logger.debug("(a.y() - c.y())")
    # logger.debug((a.y() - c.y()))
    # logger.debug("(b.x() - c.x())")
    # logger.debug((b.x() - c.x()))
    # logger.debug("(b.y() - c.y())")
    # logger.debug((b.y() - c.y()))
    # logger.debug("(c.x() - a.x())")
    # logger.debug((c.x() - a.x()))
    return abs(( ((a.y() - c.y()) * (b.x() - c.x())) \
                    + ((b.y() - c.y()) * (c.x() - a.x())) ) / 2 )
    

# IKボーンを間引きする
def decimate_bone_ik_frames_array(base_dir, bone_name_array, idecimation, ddecimation):

    base_ik_x = []
    base_ik_y = []
    base_ik_z = []
    for bf in bone_frame_dic[bone_name_array[0]]:
        base_ik_x.append(bf.position.x())
        base_ik_y.append(bf.position.y())
        base_ik_z.append(bf.position.z())

    # フィットさせたフレーム情報
    fit_frames = []
    fit_ik_x_points = []
    fit_ik_y_points = []
    fit_ik_z_points = []
    fit_ik_rotation = []

    # 同一キーフラグ
    is_samed = False
    for n in range(len(base_ik_x)):
        logger.debug("decimate_bone_ik_frames_array n={0} ik={1} ------------- ".format(n, bone_name_array[0]))
        
        # 最初は固定登録
        if n == 0:
            fit_frames.append(n)
            fit_ik_x_points.append(base_ik_x[n])
            fit_ik_y_points.append(base_ik_y[n])
            fit_ik_z_points.append(base_ik_z[n])
            fit_ik_rotation.append(bone_frame_dic[bone_name_array[0]][n].rotation)
            continue
        
        if fit_ik_x_points[-1] == base_ik_x[n] and fit_ik_y_points[-1] == base_ik_y[n]  and fit_ik_z_points[-1] == base_ik_z[n]:
            # 前回キーとまったく同じ場合、スキップ
            logger.debug("同一キースキップ")
            is_samed = True
            continue
        elif is_samed:
            # 同一フラグの後、IKの動きがある場合、前回のキーフレーム登録
            fit_frames.append(n-1)
            fit_ik_x_points.append(fit_ik_x_points[-1])
            fit_ik_y_points.append(fit_ik_y_points[-1])
            fit_ik_z_points.append(fit_ik_z_points[-1])
            fit_ik_rotation.append(fit_ik_rotation[-1])
            
            # フラグを落とす
            is_samed = False


        # # テストで今回のキーフレームも登録
        # fit_frames.append(n)
        # fit_ik_x_points.append(base_ik_x[n])
        # fit_ik_y_points.append(base_ik_y[n])
        # fit_ik_z_points.append(base_ik_z[n])
        # fit_ik_rotation.append(bone_frame_dic[bone_name_array[0]][n].rotation)
            
        # 前のキーフレームのXYZ（2点）
        prev_x1 = np.array(fit_ik_x_points[-2:])
        prev_y1 = np.array(fit_ik_y_points[-2:])
        prev_z1 = np.array(fit_ik_z_points[-2:])

        # logger.debug("prev_x1 {0}".format(prev_x1))
        # logger.debug("prev_y1 {0}".format(prev_y1))
        # logger.debug("prev_z1 {0}".format(prev_z1))
        
        # 登録対象キーフレームのXYZ
        now_x1 = np.array(base_ik_x[0+n:3+n])
        now_y1 = np.array(base_ik_y[0+n:3+n])
        now_z1 = np.array(base_ik_z[0+n:3+n])
        
        # 登録対象キーフレームのXYZ
        now_long_x1 = np.array(base_ik_x[0+n:6+n])
        now_long_y1 = np.array(base_ik_y[0+n:6+n])
        now_long_z1 = np.array(base_ik_z[0+n:6+n])
        
        # # 次の登録対象キーフレームのXYZ
        # next_x1 = np.array(base_ik_x[3+n:6+n])
        # next_y1 = np.array(base_ik_y[3+n:6+n])
        # next_z1 = np.array(base_ik_z[3+n:6+n])
        
        # # logger.debug("now_x1")
        # # logger.debug(now_x1)
        # # logger.debug("now_y1")
        # # logger.debug(now_y1)
        # # logger.debug("now_z1")
        # # logger.debug(now_z1)
    
        if len(prev_x1) <= 1 or ( np.average(prev_x1) == prev_x1[0] and np.average(prev_y1) == prev_y1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
            prev_poly_xy_angle = 0
        else:
            # 前のキーフレームの近似直線(XY)
            prev_poly_xy_fit1 = np.polyfit(prev_x1, prev_y1, 1)
            # prev_poly_xy_fit1_y_func = np.poly1d(prev_poly_xy_fit1)
            # 近似直線の角度
            prev_poly_xy_angle = np.rad2deg(np.arctan(prev_poly_xy_fit1[0]))

        logger.debug("prev_poly_xy_angle={0}".format(prev_poly_xy_angle))
            
        if len(now_x1) <= 1 or ( np.average(now_x1) == now_x1[0] and np.average(now_y1) == now_y1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
            now_poly_xy_angle = 0
        else:
            # 登録対象キーフレームの近似直線(XY)
            now_poly_xy_fit1 = np.polyfit(now_x1, now_y1, 1)
            # now_poly_xy_fit1_y_func = np.poly1d(now_poly_xy_fit1)
            # 近似直線の角度
            now_poly_xy_angle = np.rad2deg(np.arctan(now_poly_xy_fit1[0]))

        logger.debug("now_poly_xy_angle={0}".format(now_poly_xy_angle))

        if len(now_long_x1) <= 1 or ( np.average(now_long_x1) == now_long_x1[0] and np.average(now_long_y1) == now_long_y1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
            now_long_poly_xy_angle = 0
        else:
            # 登録対象長めキーフレームの近似直線(XY)
            now_long_poly_xy_fit1 = np.polyfit(now_long_x1, now_long_y1, 1)
            # now_long_poly_xy_fit1_y_func = np.poly1d(now_long_poly_xy_fit1)
            # 近似直線の角度
            now_long_poly_xy_angle = np.rad2deg(np.arctan(now_long_poly_xy_fit1[0]))

        logger.debug("now_long_poly_xy_angle={0}".format(now_long_poly_xy_angle))

        # if len(next_x1) <= 1 or ( np.average(next_x1) == next_x1[0] and np.average(next_y1) == next_y1[0] ):
        #     # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
        #     next_poly_xy_angle = 0
        # else:
        #     # 次登録対象キーフレームの近似直線(XY)
        #     next_poly_xy_fit1 = np.polyfit(next_x1, next_y1, 1)
        #     # next_poly_xy_fit1_y_func = np.poly1d(next_poly_xy_fit1)
        #     # 近似直線の角度
        #     next_poly_xy_angle = np.rad2deg(np.arctan(next_poly_xy_fit1[0]))

        # logger.debug("next_poly_xy_angle={0}".format(next_poly_xy_angle))

        # 角度差分
        diff_prev_xy = abs(np.diff([prev_poly_xy_angle, now_poly_xy_angle]))
        diff_long_xy = abs(np.diff([now_long_poly_xy_angle, now_poly_xy_angle]))
        # diff_next_xy = abs(np.diff([now_poly_xy_angle, next_poly_xy_angle]))

        logger.debug("diff_prev_xy={0}".format(diff_prev_xy))
        logger.debug("diff_long_xy={0}".format(diff_long_xy))
        # logger.debug("diff_next_xy={0}".format(diff_next_xy))
    
        if len(prev_x1) <= 1 or ( np.average(prev_x1) == prev_x1[0] and np.average(prev_z1) == prev_z1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
            prev_poly_xz_angle = 0
        else:
            # 前のキーフレームの近似直線(XZ)
            prev_poly_xz_fit1 = np.polyfit(prev_x1, prev_z1, 1)
            # prev_poly_xz_fit1_y_func = np.poly1d(prev_poly_xz_fit1)
            # 近似直線の角度
            prev_poly_xz_angle = np.rad2deg(np.arctan(prev_poly_xz_fit1[0]))

        logger.debug("prev_poly_xz_angle={0}".format(prev_poly_xz_angle))
        
        if len(now_x1) <= 1 or ( np.average(now_x1) == now_x1[0] and np.average(now_z1) == now_z1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
            now_poly_xz_angle = 0
        else:
            # 登録対象キーフレームの近似直線(XZ)
            now_poly_xz_fit1 = np.polyfit(now_x1, now_z1, 1)
            # now_poly_xz_fit1_y_func = np.poly1d(now_poly_xz_fit1)
            # 近似直線の角度
            now_poly_xz_angle = np.rad2deg(np.arctan(now_poly_xz_fit1[0]))

        logger.debug("now_poly_xz_angle {0}".format(now_poly_xz_angle))
        
        if len(now_long_x1) <= 1 or ( np.average(now_long_x1) == now_long_x1[0] and np.average(now_long_z1) == now_long_z1[0] ):
            # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
            now_long_poly_xz_angle = 0
        else:
            # 登録対象長めキーフレームの近似直線(XZ)
            now_long_poly_xz_fit1 = np.polyfit(now_long_x1, now_long_z1, 1)
            # now_long_poly_xz_fit1_y_func = np.poly1d(now_long_poly_xz_fit1)
            # 近似直線の角度
            now_long_poly_xz_angle = np.rad2deg(np.arctan(now_long_poly_xz_fit1[0]))

        logger.debug("now_long_poly_xz_angle {0}".format(now_long_poly_xz_angle))

        # if len(next_x1) <= 1 or ( np.average(next_x1) == next_x1[0] and np.average(next_z1) == next_z1[0] ):
        #     # 1つしか値がない場合、もしくは全部同じ値の場合、とりあえず0    
        #     next_poly_xz_angle = 0
        # else:
        #     # 次登録対象キーフレームの近似直線(XZ)
        #     next_poly_xz_fit1 = np.polyfit(next_x1, next_z1, 1)
        #     # next_poly_xz_fit1_y_func = np.poly1d(next_poly_xz_fit1)
        #     # 近似直線の角度
        #     next_poly_xz_angle = np.rad2deg(np.arctan(next_poly_xz_fit1[0]))

        # logger.debug("next_poly_xz_angle {0}".format(next_poly_xz_angle))
        
        # 角度差分
        diff_prev_xz = abs(np.diff([prev_poly_xz_angle, now_poly_xz_angle]))
        diff_long_xz = abs(np.diff([now_long_poly_xz_angle, now_poly_xz_angle]))
        # diff_next_xz = abs(np.diff([now_poly_xz_angle, next_poly_xz_angle]))
    
        # logger.debug("diff_prev_xz {0}".format(diff_prev_xz))
        # logger.debug("diff_long_xz {0}".format(diff_long_xz))
        # logger.debug("diff_next_xz {0}".format(diff_next_xz))

        if ( diff_prev_xy > 20 and diff_long_xy > 20) or (diff_prev_xz > 20 and diff_long_xz > 20) :
            # 前回と今回の角度の差が規定より大きい場合
            diff_x = abs(np.diff([fit_ik_x_points[-1], base_ik_x[n]]))
            diff_y = abs(np.diff([fit_ik_y_points[-1], base_ik_y[n]]))
            diff_z = abs(np.diff([fit_ik_z_points[-1], base_ik_z[n]]))

            logger.debug("diff_x={0}, diff_y={1}. diff_z={2}".format(diff_x, diff_y, diff_z))

            if diff_x >= idecimation or diff_y >= idecimation or diff_z >= idecimation :
                # IKの動きがある場合、キーフレーム登録
                fit_frames.append(n)
                fit_ik_x_points.append(base_ik_x[n])
                fit_ik_y_points.append(base_ik_y[n])
                fit_ik_z_points.append(base_ik_z[n])
                fit_ik_rotation.append(bone_frame_dic[bone_name_array[0]][n].rotation)
        # else:
        #     if diff_prev_xy < 1  and diff_prev_xz < 1 and ( diff_next_xy > 20 or diff_next_xz > 20 ):

        #         logger.debug("diff_prev_xy={0}, diff_prev_xz={1}, diff_next_xy={2}, diff_next_xz={3}".format(diff_prev_xy, diff_prev_xz, diff_next_xy, diff_next_xz))
                
        #         # 前回からほとんど動いていなくて、かつ次に動きがある場合、前回のをコピー
        #         fit_frames.append(n)
        #         fit_ik_x_points.append(fit_ik_x_points[-1])
        #         fit_ik_y_points.append(fit_ik_y_points[-1])
        #         fit_ik_z_points.append(fit_ik_z_points[-1])
        #         fit_ik_rotation.append(fit_ik_rotation[-1])

        else:
            if is_regist_rotation_frame(n, bone_name_array[0], fit_frames, ddecimation):

                logger.debug("is_regist_rotation_frame: true")
                
                # 前回と今回の移動角度の差が規定より小さく、回転角度が既定以上の場合、追加登録
                # 位置は前回のをコピーする
                fit_frames.append(n)
                fit_ik_x_points.append(base_ik_x[n])
                fit_ik_y_points.append(base_ik_y[n])
                fit_ik_z_points.append(base_ik_z[n])
                fit_ik_rotation.append(bone_frame_dic[bone_name_array[0]][n].rotation)
            else:
                for bone_name in bone_name_array[1:]:
                    if is_regist_rotation_frame(n, bone_name, fit_frames, ddecimation) and fit_frames[-1] + 1 < n:
                        logger.debug("is_regist_rotation_frame: true name={0}".format(bone_name))

                        # 後続の回転ボーンが既定角度以上の場合、追加登録
                        fit_frames.append(n)
                        fit_ik_x_points.append(base_ik_x[n])
                        fit_ik_y_points.append(base_ik_y[n])
                        fit_ik_z_points.append(base_ik_z[n])
                        fit_ik_rotation.append(bone_frame_dic[bone_name_array[0]][n].rotation)
                        break

        if n < len(base_ik_x) - 2 and fit_frames[-1] != n:
            # 終了間際でない・計算で対象になっていない場合、かつ今回と次回が同じ場合、今回を登録する
            if base_ik_x[n] == base_ik_x[n+1] == base_ik_x[n+2] \
                and base_ik_y[n] == base_ik_y[n+1] == base_ik_y[n+2]  and base_ik_z[n] == base_ik_z[n+1] == base_ik_z[n+2]:
                fit_frames.append(n)
                fit_ik_x_points.append(base_ik_x[n])
                fit_ik_y_points.append(base_ik_y[n])
                fit_ik_z_points.append(base_ik_z[n])
                fit_ik_rotation.append(bone_frame_dic[bone_name_array[0]][n].rotation)
                
        
        # if n > 35:
        #     sys.exit()

    logger.debug(len(fit_ik_x_points))

    # IKボーンを再登録
    ik_newbfs = []
    for n, frame in enumerate(fit_frames):
        # logger.debug("copy n={0}, frame={1}".format(n, frame))
        # 一旦オリジナルをコピー
        ik_bf = bone_frame_dic[bone_name_array[0]][frame]
        # logger.debug("ik_bf")
        # logger.debug(ik_bf.position)
        # logger.debug(ik_bf.rotation)
        # logger.debug("fit_ik_x_points = {0}".format(len(fit_ik_x_points)))
        # logger.debug("fit_ik_y_points = {0}".format(len(fit_ik_y_points)))
        # logger.debug("fit_ik_z_points = {0}".format(len(fit_ik_z_points)))
        # logger.debug("fit_ik_rotation = {0}".format(len(fit_ik_rotation)))
        # 設定値をコピー
        ik_bf.frame = frame
        ik_bf.position.setX(fit_ik_x_points[n])
        ik_bf.position.setY(fit_ik_y_points[n])
        ik_bf.position.setZ(fit_ik_z_points[n])
        ik_bf.rotation = fit_ik_rotation[n]
        ik_newbfs.append(ik_bf)

    # 新しいフレームリストを登録する
    bone_frame_dic[bone_name_array[0]] = ik_newbfs

    # IK以降の回転ボーンを再登録
    for bone_name in bone_name_array[1:]:
        newbfs = []
        for n in fit_frames:
            # logger.debug("n={0}, bone_name={1}, len={2}".format(n, bone_name, len(bone_frame_dic[bone_name])))
            newbfs.append(bone_frame_dic[bone_name][n])
        
        # 新しいフレームリストを登録する
        bone_frame_dic[bone_name] = newbfs


    # if level[verbose] == logging.DEBUG:
    #     fit_start = 0
    #     for n in range(0, len(base_ik_x), 50):
    #         plt.cla()
    #         plt.clf()
    #         fig, (axY, axZ) = plt.subplots(ncols=2, figsize=(15, 15))
    #         axY.plot(np.array(base_ik_x[n:n+50]) , np.array(base_ik_y[n:n+50]), lw=2)
    #         axZ.plot(np.array(base_ik_x[n:n+50]) , np.array(base_ik_z[n:n+50]), lw=2)

    #         fit_end = 0
    #         for m in range(fit_start, len(fit_frames)):
    #             if fit_frames[m] >= n+50:
    #                 fit_end = m
    #                 break

    #         axY.plot(np.array(fit_ik_x_points[fit_start:fit_end]) , np.array(fit_ik_y_points[fit_start:fit_end]), c="#CC0000")
    #         axZ.plot(np.array(fit_ik_x_points[fit_start:fit_end]) , np.array(fit_ik_z_points[fit_start:fit_end]), c="#CC0000")

    #         fit_start = fit_end + 1

    #         plotName = "{0}/plot_{1}_{2:05d}.png".format(base_dir, bone_name_array[0], n)
    #         plt.savefig(plotName)
    #         plt.close()


def position_multi_file_to_vmd(position_file, position_gan_file, upright_file, vmd_file, smoothed_file, bone_csv_file, depth_file, start_frame_file, center_xy_scale, center_z_scale, xangle, mdecimation, idecimation, ddecimation, alignment, is_ik, heelpos, smooth_times, upright_target):
    positions_multi = read_positions_multi(position_file)
    
    # 3dpose-gan がない場合はNone
    if os.path.exists(position_gan_file):
        positions_gan_multi = read_positions_multi(position_gan_file)
    else:
        positions_gan_multi = None

    position_list_to_vmd_multi(positions_multi, positions_gan_multi, upright_file, vmd_file, smoothed_file, bone_csv_file, depth_file, start_frame_file, center_xy_scale, center_z_scale, xangle, mdecimation, idecimation, ddecimation, alignment, is_ik, heelpos, smooth_times, upright_target)
    
def make_showik_frames(is_ik):
    onoff = 1 if is_ik == True else 0

    frames = []
    sf = VmdShowIkFrame()
    sf.show = 1
    sf.ik.append(VmdInfoIk(b'\x8d\xb6\x91\xab\x82\x68\x82\x6a', onoff)) # '左足ＩＫ'
    sf.ik.append(VmdInfoIk(b'\x89\x45\x91\xab\x82\x68\x82\x6a', onoff)) # '右足ＩＫ'
    sf.ik.append(VmdInfoIk(b'\x8d\xb6\x82\xc2\x82\xdc\x90\xe6\x82\x68\x82\x6a', onoff)) # '左つまＩＫ'
    sf.ik.append(VmdInfoIk(b'\x89\x45\x82\xc2\x82\xdc\x90\xe6\x82\x68\x82\x6a', onoff)) # '右つまＩＫ'
    frames.append(sf)
    return frames

def get_file_encoding(file_path):

    try: 
        f = open(file_path, "rb")
        fbytes = f.read()
        f.close()
    except:
        raise Exception("unknown encoding!")
        
    codelst = ('utf_8', 'shift-jis')
    
    for encoding in codelst:
        try:
            fstr = fbytes.decode(encoding) # bytes文字列から指定文字コードの文字列に変換
            fstr = fstr.encode('utf-8') # uft-8文字列に変換
            # 問題なく変換できたらエンコードを返す
            logger.debug("%s: encoding: %s", file_path, encoding)
            return encoding
        except:
            pass
            
    raise Exception("unknown encoding!")
    


if __name__ == '__main__':
    import sys
    if (len(sys.argv) < 2):
        usage(sys.argv[0])

    parser = argparse.ArgumentParser(description='3d-pose-baseline to vmd')
    parser.add_argument('-t', '--target', dest='target', type=str,
                        help='target directory (3d-pose-baseline-vmd)')
    parser.add_argument('-u', '--upright-target', dest='upright_target', type=str,
                        default='',
                        help='upright target directory')
    parser.add_argument('-b', '--bone', dest='bone', type=str,
                        help='target model bone csv')
    parser.add_argument('-v', '--verbose', dest='verbose', type=int,
                        default=2,
                        help='logging level')
    parser.add_argument('-c', '--center-xyscale', dest='centerxy', type=int,
                        default=0,
                        help='center scale')
    parser.add_argument('-z', '--center-z-scale', dest='centerz', type=float,
                        default=0,
                        help='center z scale')
    parser.add_argument('-s', '--smooth-times', dest='smooth_times', type=int,
                        default=1,
                        help='smooth times')
    parser.add_argument('-x', '--x-angle', dest='xangle', type=int,
                        default=0,
                        help='global x angle correction')
    parser.add_argument('-d', '--born-decimation-angle', dest='ddecimation', type=int,
                        default=0,
                        help='born frame decimation angle')
    parser.add_argument('-m', '--center-move-born-decimation', dest='mdecimation', type=float,
                        default=0,
                        help='born frame center decimation move')
    parser.add_argument('-i', '--ik-move-born-decimation', dest='idecimation', type=float,
                        default=0,
                        help='born frame ik decimation move')
    parser.add_argument('-a', '--decimation-alignment', dest='alignment', type=int,
                        default=1,
                        help='born frame decimation alignment')
    parser.add_argument('-k', '--leg-ik', dest='legik', type=int,
                        default=1,
                        help='leg ik')
    parser.add_argument('-e', '--heel position', dest='heelpos', type=float,
                        default=0,
                        help='heel position correction')
    args = parser.parse_args()

    # resultディレクトリだけ指定させる
    base_dir = args.target

    is_alignment = True if args.alignment == 1 else False

    is_ik = True if args.legik == 1 else False

    # 入力と出力のファイル名は固定
    position_file = base_dir + "/pos.txt"
    smoothed_file = base_dir + "/smoothed.txt"
    depth_file = base_dir + "/depth.txt"
    start_frame_file = base_dir + "/start_frame.txt"

    # 3dpose-gan のposファイル。（ない可能性あり）
    position_gan_file = base_dir + "/pos_gan.txt"

    suffix = ""
    if os.path.exists(position_gan_file) == False:
        suffix = "_ganなし"
    
    if os.path.exists(depth_file) == False:
        suffix = "{0}_depthなし".format(suffix)
    
    if is_ik == False:
        suffix = "{0}_FK".format(suffix)
    
    # 踵位置補正
    suffix = "{0}_h{1}".format(suffix, str(args.heelpos))
    
    # センターXY
    suffix = "{0}_xy{1}".format(suffix, str(args.centerxy))

    # センターZ        
    suffix = "{0}_z{1}".format(suffix, str(args.centerz))
    
    # グローバルX補正
    suffix = "{0}_gx{1}".format(suffix, str(args.xangle))
    
    # 円滑化回数
    suffix = "{0}_s{1}".format(suffix, str(args.smooth_times))

    # 間引き
    suffix = "{0}_c{1}".format(suffix, str(args.mdecimation))
    
    
    if args.mdecimation != 0 and args.idecimation != 0 and args.ddecimation != 0 and is_alignment == False:
        suffix = "{0}_揃えなし".format(suffix)

    vmd_file = "{0}/output_{1:%Y%m%d_%H%M%S}_[uDDDD]{2}.vmd".format(base_dir, datetime.datetime.now(), suffix)

    #直立インデックスファイル
    upright_file = open("{0}/upright.txt".format(base_dir), 'w')

    # ログレベル設定
    logger.setLevel(level[args.verbose])

    verbose = args.verbose

    # if os.path.exists('predictor/shape_predictor_68_face_landmarks.dat'):
    #     head_rotation = 

    position_multi_file_to_vmd(position_file, position_gan_file, upright_file, vmd_file, smoothed_file, args.bone, depth_file, start_frame_file, args.centerxy, args.centerz, args.xangle, args.mdecimation, args.idecimation, args.ddecimation, is_alignment, is_ik, args.heelpos, args.smooth_times, args.upright_target)
