#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
import os
import numpy as np
import logging
from decimal import Decimal
from matplotlib import pyplot as plt
from collections import Counter

logger = logging.getLogger("pos2vmd_multi").getChild(__name__)

# 間引き処理
def decimate(bone_frame_dic, vmd_file, is_upper2_body, is_groove, is_alignment, is_ik, mdecimation, idecimation, ddecimation):
    
    if mdecimation > 0 or idecimation > 0 or ddecimation > 0:
        
        base_dir = os.path.dirname(vmd_file)

        if is_alignment == True:
            logger.info("揃えて間引き開始")
            # 揃えて間引き
            decimate_bone_center_frames_array(bone_frame_dic, base_dir, is_groove, mdecimation)
            
            if is_ik:                
                decimate_bone_ik_frames_array(bone_frame_dic, base_dir, ["左足ＩＫ", "左足"], idecimation, ddecimation)
                decimate_bone_ik_frames_array(bone_frame_dic, base_dir, ["右足ＩＫ", "右足"], idecimation, ddecimation)
            else:
                decimate_bone_ik_frames_array(bone_frame_dic, base_dir, ["左足", "左ひざ"], idecimation, ddecimation)
                decimate_bone_ik_frames_array(bone_frame_dic, base_dir, ["右足", "右ひざ"], idecimation, ddecimation)
                
            # decimate_bone_rotation_frames_array(bone_frame_dic, ["上半身"], ddecimation)
            # decimate_bone_rotation_frames_array(bone_frame_dic, ["下半身"], ddecimation)
            if is_upper2_body:
                decimate_bone_rotation_frames_array(bone_frame_dic, ["上半身", "上半身2", "下半身"], ddecimation)
            else:
                decimate_bone_rotation_frames_array(bone_frame_dic, ["上半身", "下半身"], ddecimation)

            decimate_bone_rotation_frames_array(bone_frame_dic, ["首", "頭"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["左ひじ", "左腕", "左肩"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["右ひじ", "右腕", "右肩"], ddecimation)
        else:
            logger.info("通常間引き開始")
            decimate_bone_center_frames_array(bone_frame_dic, base_dir, is_groove, mdecimation)

            if is_ik:
                decimate_bone_ik_frames_array(bone_frame_dic, base_dir, ["左足ＩＫ"], idecimation, ddecimation)
                decimate_bone_ik_frames_array(bone_frame_dic, base_dir, ["右足ＩＫ"], idecimation, ddecimation)
            else:
                decimate_bone_rotation_frames_array(bone_frame_dic, ["左ひざ"], ddecimation)
                decimate_bone_rotation_frames_array(bone_frame_dic, ["右ひざ"], ddecimation)
                
            if is_upper2_body:
                decimate_bone_rotation_frames_array(bone_frame_dic, ["上半身2"], ddecimation)

            decimate_bone_rotation_frames_array(bone_frame_dic, ["上半身"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["下半身"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["左足"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["右足"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["首"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["頭"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["左ひじ"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["左腕"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["左肩"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["右ひじ"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["右腕"], ddecimation)
            decimate_bone_rotation_frames_array(bone_frame_dic, ["右肩"], ddecimation)

# IKボーンを間引きする
def decimate_bone_ik_frames_array(bone_frame_dic, base_dir, bone_name_array, idecimation, ddecimation):

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
            if is_regist_rotation_frame(bone_frame_dic, n, bone_name_array[0], fit_frames, ddecimation):

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
                    if is_regist_rotation_frame(bone_frame_dic, n, bone_name, fit_frames, ddecimation) and fit_frames[-1] + 1 < n:
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


# センターボーンを間引きする
def decimate_bone_center_frames_array(bone_frame_dic, base_dir, is_groove, mdecimation):

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
def decimate_bone_rotation_frames_array(bone_frame_dic, bone_name_array, ddecimation):

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
            if is_regist_rotation_frame(bone_frame_dic, n, bone_name, fit_frames, ddecimation):
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
def is_regist_rotation_frame(bone_frame_dic, n, bone_name, fit_frames, ddecimation):
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
