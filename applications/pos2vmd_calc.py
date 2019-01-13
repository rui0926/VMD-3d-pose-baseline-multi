#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
from PyQt5.QtGui import QQuaternion, QVector4D, QVector3D, QMatrix4x4
import logging
import csv
import numpy as np
import math
import copy

import pos2vmd_utils

logger = logging.getLogger("__main__").getChild(__name__)

# 全身で最も直立している姿勢をいくつか返す
def calc_upright_body(bone_frame_dic):
    return calc_upright_bones(bone_frame_dic, ["上半身", "上半身2", "下半身", "左足", "左ひざ", "右足", "右ひざ"])

# 最も直立している姿勢をいくつか返す
def calc_upright_bones(bone_frame_dic, target_bones):

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

def calc_upright_bone(bone_frame_dic, bone_name):

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


# IKの計算
def calc_IK(bone_frame_dic, bone_csv_file, smoothed_2d, depth_all_frames, upright_idxs, heelpos):
    logger.debug("bone_csv_file: "+ bone_csv_file)

    upright_idx = upright_idxs[0]

    # ボーンファイルを開く
    with open(bone_csv_file, "r", encoding=pos2vmd_utils.get_file_encoding(bone_csv_file)) as bf:
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
    center_upright_2d_y = (smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].y() + smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].y()) / 2

    # 前回フレーム
    prev_left_frame = 0
    prev_right_frame = 0

    for n in range(len(bone_frame_dic["左足"])):
        logger.debug("足IK計算 frame={0}".format(n))
        # logger.debug("右足踵={0}, 左足踵={1}".format(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]], smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]]))

        # logger.debug("前回左x={0}, 今回左x={1}, 差分={2}".format(smoothed_2d[prev_left_frame][4].x(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]].x(), abs(np.diff([smoothed_2d[prev_left_frame][4].x(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]].x()]))))
        # logger.debug("前回左y={0}, 今回左y={1}, 差分={2}".format(smoothed_2d[prev_left_frame][4].y(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]].y(), abs(np.diff([smoothed_2d[prev_left_frame][4].y(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]].y()]))))

        #左足IK
        if n > 0 and abs(np.diff([smoothed_2d[prev_left_frame][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]].x(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]].x()])) < 5 and abs(np.diff([smoothed_2d[prev_left_frame][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]].y(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]].y()])) < 5:
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

        # logger.debug("前回右x={0}, 今回右x={1}, 差分={2}".format(smoothed_2d[prev_left_frame][3].x(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]].x(), abs(np.diff([smoothed_2d[prev_left_frame][3].x(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]].x()]))))
        # logger.debug("前回右y={0}, 今回右y={1}, 差分={2}".format(smoothed_2d[prev_left_frame][3].y(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]].y(), abs(np.diff([smoothed_2d[prev_left_frame][3].y(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]].y()]))))
            
        # 右足IK
        if n > 0 and abs(np.diff([smoothed_2d[prev_right_frame][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]].x(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]].x()])) < 5 and abs(np.diff([smoothed_2d[prev_right_frame][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]].y(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]].y()])) < 5:
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

            # 足IKを地表にあげる
            left_ankle_pos.setY( left_ankle_pos.y() - ankle_pos_max )
            right_ankle_pos.setY( right_ankle_pos.y() - ankle_pos_max )

            # FIXME センターががくがくする？要調査
            bone_frame_dic["センター"][n].position.setY( bone_frame_dic["センター"][n].position.y() - ankle_pos_max )
            
            # logger.debug("center.y2:{0}".format(bone_frame_dic["センター"][n].position.y()))    

            # X回転もさせず、接地させる
            left_ik_rotation = QQuaternion.fromEulerAngles(0, left_ik_rotation.toEulerAngles().y(), left_ik_rotation.toEulerAngles().z() )
            right_ik_rotation = QQuaternion.fromEulerAngles(0, right_ik_rotation.toEulerAngles().y(), right_ik_rotation.toEulerAngles().z() )

        # FIXME ジャンプしてる時と浮いてる時の区別がつかないので、一旦保留        
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
            left_ik_rotation = QQuaternion.fromEulerAngles(0, left_ik_rotation.toEulerAngles().y(), left_ik_rotation.toEulerAngles().z() )

        if (right_ankle_pos.y() < 0 and left_ankle_pos.y() >= 0):
            # 右足だけの場合マイナス値は0に補正
            right_ankle_pos.setY(0)

            # X回転もさせず、接地させる
            right_ik_rotation = QQuaternion.fromEulerAngles(0, right_ik_rotation.toEulerAngles().y(), right_ik_rotation.toEulerAngles().z() )

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
def calc_center_z(bone_frame_dic, smoothed_2d, depths, start_frame, upright_idxs, center_xy_scale, center_z_scale, target_upright_idx, target_upright_depth):

    if center_z_scale == 0:
        return

    # 直立インデックス 
    upright_idx = upright_idxs[0]

    # 全フレームの推定深度
    depth_all_frames = []

    # 添え字と腰深度の配列
    depth_indexes = np.array(depths)[:, pos2vmd_utils.DEPTH_INDEX["index"]:pos2vmd_utils.DEPTH_INDEX["index"]+1].flatten()
    waist_depth_values = np.array(depths)[:, pos2vmd_utils.DEPTH_INDEX["Wrist"]:pos2vmd_utils.DEPTH_INDEX["Wrist"]+1].flatten()

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
    upright_upper_length = (((smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].y() + smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].y() ) / 2) - smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]].y()) / center_xy_scale    

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
        tu_upper_lengths.append( ((( smoothed_2d[d_tunidx][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].y() + smoothed_2d[d_tunidx][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].y() ) / 2) - smoothed_2d[d_tunidx][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]].y()) / center_xy_scale )
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

    target_diff_z = 0
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
        bone_frame_dic["センター"][nn - start_frame].position.setZ(float(center_z))
        
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

                bone_frame_dic["センター"][m - start_frame].position.setZ(float(center_z))
                
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
def calc_center(bone_frame_dic, smoothed_2d, bone_csv_file, upright_idxs, center_xy_scale, center_z_scale, heelpos, target_upright_idx, target_start_pos):

    if center_xy_scale == 0:
        return

    logger.debug("bone_csv_file: "+ bone_csv_file)

    # 直立インデックス
    upright_idx = upright_idxs[0]    

    # ボーンファイルを開く
    with open(bone_csv_file, "r",  encoding=pos2vmd_utils.get_file_encoding(bone_csv_file)) as bf:
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
    bone_upright_area = pos2vmd_utils.calc_triangle_area(neck_3d, right_leg_3d, left_leg_3d)

    # logger.debug("smoothed_2d[upright_idx]")
    # logger.debug(smoothed_2d[upright_idx])

    # 直立フレームの三角形面積
    smoothed_upright_area = pos2vmd_utils.calc_triangle_area(smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]], smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]], smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]])

    # logger.debug("upright_area")
    # logger.debug(smoothed_upright_area)

    # ボーンと映像の三角形比率(スケール調整あり)
    upright_xy_scale = bone_upright_area / smoothed_upright_area * center_xy_scale

    # logger.debug("upright_scale")
    # logger.debug(upright_xy_scale)

    # 直立フレームの左足と右足の位置のY平均
    upright_leg_avg = abs((smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].y() + smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].y()) / 2)

    # 直立フレームの首・左足と右足の位置のX平均
    upright_neck_leg_x_avg = (smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]].x() + smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].x() + smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].x()) / 3

    # 直立フレームの左足首と右足首の位置の平均
    upright_ankle_avg = abs((smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]].y() + smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]].y()) / 2)

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
    
    upright_adjust_neck_leg_x_avg = 0
    if target_start_pos["center"] != QVector3D():
        # 0F目で調整用POSが指定されているの場合、X差分を取得
        upright_adjust_neck_leg_x_avg = (target_start_pos["Neck"].x() + target_start_pos["RHip"].x() + target_start_pos["LHip"].x()) / 3
        logger.info("upright_adjust_neck_leg_x_avg %s", upright_adjust_neck_leg_x_avg)

    for n, smoothed in enumerate(smoothed_2d):
        logger.debug("センター計算 frame={0}".format(n))

        # 左足と右足の位置の小さい方
        ankle_min = np.min([smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RAnkle"]].y(), smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LAnkle"]].y()])

        # logger.debug("ankle_min")
        # logger.debug(ankle_min)

        # logger.debug("ankle_min * upright_ankle_scale")
        # logger.debug(ankle_min * upright_ankle_scale)

        # 左足と右足の位置の平均
        leg_avg = abs((smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].y() + smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].y()) / 2)
        
        # 足の上下差
        leg_diff = upright_leg_avg - leg_avg

        # Y軸移動(とりあえずセンター固定)
        center_y = (leg_diff * upright_xy_scale) - (ankle_min * upright_ankle_scale)

        # 踵補正を入れて設定する
        bone_frame_dic["センター"][n].position.setY(center_y + heelpos)
        # bone_frame_dic["センター"][n].position.setY((leg_diff * upright_xy_scale))
        
        # 首・左足・右足の中心部分をX軸移動
        x_avg = ((smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]].x() + smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].x() + smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].x()) / 3) \
                    - upright_neck_leg_x_avg + upright_adjust_neck_leg_x_avg
        center_x = x_avg * upright_xy_scale

        bone_frame_dic["センター"][n].position.setX(center_x)

        logger.debug("center {0} x={1}, y={2}".format(n, center_x, center_y))

        # 現在の映像の三角形面積
        # now_smoothed_area = calc_triangle_area(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]], smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]], smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]])

        # logger.debug("smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]]")
        # logger.debug(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]])

        # logger.debug("smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]]")
        # logger.debug(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]])

        # logger.debug("smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]]")
        # logger.debug(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]])

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
        # smoothed_now_area = calc_triangle_area(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]], smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]], smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]])

        # logger.debug("smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]]")    
        # logger.debug(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]])
        # logger.debug("smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]]")    
        # logger.debug(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]])
        # logger.debug("smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]]")    
        # logger.debug(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]])

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
        # smoothed_upright_slope_left_leg = calc_slope_point(smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]], rx * -1, ry * -1, rz * -1)
        # # 傾いたところの頂点：右足
        # smoothed_upright_slope_right_leg = calc_slope_point(smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]], rx * -1, ry * -1, rz * -1)

        # # 傾きを反転させた直立面積
        # smoothed_upright_slope_area = calc_triangle_area(smoothed_upright_slope_neck, smoothed_upright_slope_left_leg, smoothed_upright_slope_right_leg)

        # logger.debug("smoothed_upright_slope_area")
        # logger.debug(smoothed_upright_slope_area)

        # logger.debug("smoothed_upright_area")
        # logger.debug(smoothed_upright_area)

        # # 直立の関節の回転分面積を現在の関節面積で割って、大きさの比率を出す
        # now_z_scale = smoothed_upright_slope_area / smoothed_upright_area

        # if n == 340 or n == 341:

        #     logger.debug("smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]]")
        #     logger.debug(smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]])

        #     logger.debug("smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]]")
        #     logger.debug(smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]])

        #     logger.debug("smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]]")
        #     logger.debug(smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]])

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
        
        # logger.debug("smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]].x()")
        # logger.debug(smoothed_2d[upright_idx][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]].x())
        
        # logger.debug("smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]].x()")
        # logger.debug(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]].x())
        
        # logger.debug("smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].x()")
        # logger.debug(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].x())
        
        # logger.debug("smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].x()")
        # logger.debug(smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].x())
        
        # logger.debug("((smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]].x() + smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].x() + smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].x()) / 3)")
        # logger.debug(((smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["Neck"]].x() + smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["RHip"]].x() + smoothed_2d[n][pos2vmd_utils.SMOOTHED_2D_INDEX["LHip"]].x()) / 3))
        
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
