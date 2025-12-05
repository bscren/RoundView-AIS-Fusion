import cv2
import numpy as np
import pandas as pd

def draw_box(add_img, x1, y1, x2, y2, color, tf):
    y15 = y1 + (y2 - y1) // 4
    x15 = x1 + (y2 - y1) // 4
    y45 = y2 - (y2 - y1) // 4
    x45 = x2 - (y2 - y1) // 4

    # 左上角
    cv2.line(add_img, (x1, y1), (x1, y15), color, tf)
    cv2.line(add_img, (x1, y1), (x15, y1), color, tf)
    # 右上角
    cv2.line(add_img, (x2, y1), (x2, y15), color, tf)
    cv2.line(add_img, (x45, y1), (x2, y1), color, tf)
    # 左下角
    cv2.line(add_img, (x1, y2), (x15, y2), color, tf)
    cv2.line(add_img, (x1, y45), (x1, y2), color, tf)
    # 右下角
    cv2.line(add_img, (x45, y2), (x2, y2), color, tf)
    cv2.line(add_img, (x2, y45), (x2, y2), color, tf)

    return add_img

def draw_line(add_img, x1, y1, x2, y2, y_deta, color, tf):
    cv2.circle(add_img, (x1, y1), tf, color, tf // 3)
    cv2.circle(add_img, (x2, y2), tf, color, tf // 3)
    cv2.line(add_img, (x1, y1 + tf), (x1, y1 + y_deta), color, tf // 2)
    cv2.line(add_img, (x1, y1 + y_deta), (x2, y1 + y_deta), color, tf // 2)
    cv2.line(add_img, (x2, y1 + y_deta), (x2, y2 - tf), color, tf // 2)
    return add_img

def draw(add_img, df_draw, tf):
    """
    Draw the trajectory of the船舶
    add_img: the image to be drawn
    df_draw: the dataframe to be drawn, include:
        ais: the AIS information, 1 for AIS-VIS, 0 for VIS
        mmsi: the MMSI information
        sog: the SOG information
        cog: the COG information
        lat: the latitude information
        lon: the longitude information
        box_x1: the x coordinate of the left top corner of the box
        box_y1: the y coordinate of the left top corner of the box
        box_x2: the x coordinate of the right bottom corner of the box
        box_y2: the y coordinate of the right bottom corner of the box
        inf_x1: the x coordinate of the left top corner of the information box
        inf_y1: the y coordinate of the left top corner of the information box
        inf_x2: the x coordinate of the right bottom corner of the information box
        inf_y2: the y coordinate of the right bottom corner of the information box
        color: the color of the box
    tf: the font thickness
    """
    length = len(df_draw)
    if length != 0:
        y1 = df_draw['box_y2'][0]
        y2 = df_draw['inf_y1'][0]
        y = y2 - y1

    i = 0
    for ind, inf in df_draw.iterrows():
        ais = inf['ais']
        mmsi = inf['mmsi']
        sog = inf['sog']
        cog = inf['cog']
        lat = inf['lat']
        lon = inf['lon']
        box_x1 = inf['box_x1']
        box_y1 = inf['box_y1']
        box_x2 = inf['box_x2']
        box_y2 = inf['box_y2']
        inf_x1 = inf['inf_x1']
        inf_y1 = inf['inf_y1']
        inf_x2 = inf['inf_x2']
        inf_y2 = inf['inf_y2']
        color = inf['color']

        add_img = draw_box(add_img, box_x1, box_y1, box_x2, box_y2, color, tf)

        if ais == 1:
            cv2.rectangle(add_img, (inf_x1, inf_y1), (inf_x2, inf_y2), color, thickness=max(tf // 3, 1), lineType=cv2.LINE_AA)
            cv2.putText(add_img, 'MMSI:{}'.format(mmsi), (inf_x1 + tf, inf_y1 + tf * 5),
                        cv2.FONT_HERSHEY_SIMPLEX, tf / 8, color, max(tf // 2, 1))
            cv2.putText(add_img, 'SOG:{}'.format(sog), (inf_x1 + tf, inf_y1 + tf * 11),
                        cv2.FONT_HERSHEY_SIMPLEX, tf / 8, color, max(tf // 2, 1))
            cv2.putText(add_img, 'COG:{}'.format(cog), (inf_x1 + tf, inf_y1 + tf * 17),
                        cv2.FONT_HERSHEY_SIMPLEX, tf / 8, color, max(tf // 2, 1))
            cv2.putText(add_img, 'LAT:{}'.format(lat), (inf_x1 + tf, inf_y1 + tf * 23),
                        cv2.FONT_HERSHEY_SIMPLEX, tf / 8, color, max(tf // 2, 1))
            cv2.putText(add_img, 'LON:{}'.format(lon), (inf_x1 + tf, inf_y1 + tf * 29),
                        cv2.FONT_HERSHEY_SIMPLEX, tf / 8, color, max(tf // 2, 1))
            add_img = draw_line(add_img, (box_x1 + box_x2) // 2, box_y2, (inf_x1 + inf_x2) // 2, inf_y1, y * (i + 1) // (length + 1), color, tf)
            i = i + 1
        else:
            cv2.rectangle(add_img, (inf_x1, inf_y1), (inf_x2, inf_y2), color, thickness=max(tf // 3, 1), lineType=cv2.LINE_AA)
    return add_img