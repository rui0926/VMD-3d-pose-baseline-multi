#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
from PyQt5.QtGui import QQuaternion, QVector4D, QVector3D, QMatrix4x4
import logging
import csv
import re
import os
import numpy as np

logger = logging.getLogger("pos2vmd_multi").getChild(__name__)

# 開始フレームを取得
def load_start_frame(start_frame_file):
    n = 0
    with open(start_frame_file, "r") as sf:
        return int(sf.readline())

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

# ファイルのエンコードを取得する
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
    