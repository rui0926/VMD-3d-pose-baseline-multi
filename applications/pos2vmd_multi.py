#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# pos2vmd.py - convert joint position data to VMD

from __future__ import print_function

from PyQt5.QtGui import QQuaternion, QVector4D, QVector3D, QMatrix4x4
import os
import re
import argparse
import logging
import datetime
import numpy as np
import csv

from VmdWriter import VmdBoneFrame, VmdInfoIk, VmdShowIkFrame, VmdWriter
import pos2vmd_utils
import pos2vmd_calc
import pos2vmd_decimation
import pos2vmd_frame
              
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("pos2vmd_multi")

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
def position_list_to_vmd_multi(positions_multi, positions_gan_multi, upright_file, vmd_file, smoothed_file, bone_csv_file, depth_file, start_frame_file, center_xy_scale, center_z_scale, xangle, mdecimation, idecimation, ddecimation, is_alignment, is_ik, heelpos, smooth_times, upright_target):
    writer = VmdWriter()
    
    # 開始フレームインデックス
    start_frame = pos2vmd_utils.load_start_frame(start_frame_file)
    logger.info("開始フレームインデックス: %d", start_frame)
    
    # 関節二次元情報を読み込み
    smoothed_2d = pos2vmd_utils.load_smoothed_2d(smoothed_file)

    # 上半身2があるかチェック
    is_upper2_body = pos2vmd_utils.is_upper2_body_bone(bone_csv_file)

    logger.info("角度計算開始")

    # 各関節角度の算出
    for frame, positions in enumerate(positions_multi):
        positions_gan = None
        if positions_gan_multi is not None:
            positions_gan = positions_gan_multi[frame]

        pos2vmd_frame.position_to_frame(bone_frame_dic, positions, positions_gan, smoothed_2d, frame, xangle, is_upper2_body)   

    logger.info("直立フレーム推定開始")

    # 体幹的に最も直立しているINDEX抽出
    upright_idxs = pos2vmd_calc.calc_upright_body(bone_frame_dic)

    logger.info(upright_idxs)

    logger.info("センター計算開始")

    # センター調整
    target_upright_idx, target_upright_depth, target_start_pos = load_upright_target(upright_target)

    # センターの計算
    pos2vmd_calc.calc_center(bone_frame_dic, smoothed_2d, bone_csv_file, upright_idxs, center_xy_scale, center_z_scale, heelpos, target_upright_idx, target_start_pos)

    depths = pos2vmd_utils.load_depth(depth_file)

    depth_all_frames = None
    if depths is not None:
        # 深度ファイルがある場合のみ、Z軸計算
        logger.info("センターZ計算開始")

        # センターZの計算
        depth_all_frames = pos2vmd_calc.calc_center_z(bone_frame_dic, smoothed_2d, depths, start_frame, upright_idxs, center_xy_scale, center_z_scale, target_upright_idx, target_upright_depth)

    # 角度をなめらかに
    smooth_angle(smooth_times, ["上半身", "上半身2", "下半身", "首", "頭", "左肩", "左腕", "左ひじ", "右肩",  "右腕", "右ひじ", "左足", "左ひざ", "右足", "右ひざ"])

    logger.info("IK計算開始")

    if is_ik:
        # IKの計算
        pos2vmd_calc.calc_IK(bone_frame_dic, bone_csv_file, smoothed_2d, depth_all_frames, upright_idxs, heelpos)
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
        s2d = smoothed_2d[0][pos2vmd_utils.SMOOTHED_2D_INDEX[key]]
        # logger.info(s2d)
        upright_file.write("{0},{1},{2},{3}".format(key, s2d.x(), s2d.y(), s2d.z()))
        upright_file.write("\n")

    upright_file.close()

    logger.info("グルーブ移管開始")

    # グルーブ移管
    is_groove = set_groove(bone_csv_file)

    # 間引き処理
    pos2vmd_decimation.decimate(bone_frame_dic, vmd_file, is_upper2_body, is_groove, is_alignment, is_ik, mdecimation, idecimation, ddecimation)

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


# 調整直立情報取得
def load_upright_target(upright_target):

    target_upright_depth = 0
    target_upright_idx = 0
    target_start_pos = {}

    # 初期値
    target_start_pos["center"] = QVector3D()
    for key in ["Neck", "RHip", "LHip", "RKnee", "LKnee", "RAnkle", "LAnkle"]:
        target_start_pos[key] = QVector3D()

    if upright_target is not None:
        path = upright_target +"/upright.txt"
        logger.debug("path: %s %s", path, os.path.exists(path))
        if os.path.exists(path):
            # 直立調整対象ファイルが存在する場合
            with open(path, "r") as bf:
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
            
    # logger.info("target_start_pos")
    # logger.info(target_start_pos)

    return target_upright_idx, target_upright_depth, target_start_pos
            
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

# センターY軸をグルーブY軸に移管
def set_groove(bone_csv_file):

    # グルーブボーンがあるか
    is_groove = False
    # ボーンファイルを開く
    with open(bone_csv_file, "r", encoding=pos2vmd_utils.get_file_encoding(bone_csv_file)) as bf:
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

def position_multi_file_to_vmd(position_file, position_gan_file, upright_file, vmd_file, smoothed_file, bone_csv_file, depth_file, start_frame_file, center_xy_scale, center_z_scale, xangle, mdecimation, idecimation, ddecimation, alignment, is_ik, heelpos, smooth_times, upright_target):
    positions_multi = pos2vmd_utils.read_positions_multi(position_file)
    
    # 3dpose-gan がない場合はNone
    if os.path.exists(position_gan_file):
        positions_gan_multi = pos2vmd_utils.read_positions_multi(position_gan_file)
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

if __name__ == '__main__':
    import sys
    if (len(sys.argv) < 13):
        logger.error("引数不足")

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

    # 調整用が指定されており、かつ処理対象と違うならば保持
    upright_target = None
    if args.upright_target != args.target and len(args.upright_target) > 0:
        upright_target = args.upright_target

    position_multi_file_to_vmd(position_file, position_gan_file, upright_file, vmd_file, smoothed_file, args.bone, depth_file, start_frame_file, args.centerxy, args.centerz, args.xangle, args.mdecimation, args.idecimation, args.ddecimation, is_alignment, is_ik, args.heelpos, args.smooth_times, upright_target)
