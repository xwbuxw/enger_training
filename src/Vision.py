import cv2
import numpy as np


def adaptive_threshold(hsv, mask):
    """计算自适应HSV阈值范围，基于目标颜色区域的均值和标准差"""
    h_values = hsv[:, :, 0][mask > 0]
    s_values = hsv[:, :, 1][mask > 0]
    v_values = hsv[:, :, 2][mask > 0]

    # 计算每个通道的均值和标准差
    h_mean, h_std = np.mean(h_values), np.std(h_values)
    s_mean, s_std = np.mean(s_values), np.std(s_values)
    v_mean, v_std = np.mean(v_values), np.std(v_values)

    param = 2
    lower_bound = np.array([max(0, h_mean - param * h_std), max(0, s_mean - param * s_std), max(0, v_mean - param * v_std)])
    upper_bound = np.array(
        [min(180, h_mean + param * h_std), min(255, s_mean + param * s_std), min(255, v_mean + param * v_std)])
    # print(lower_bound.astype(int), upper_bound.astype(int))
    return lower_bound.astype(int), upper_bound.astype(int)


def search_obj(frame, color, adaptive=True, filter=False):
    center_point = [0, 0]
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h_values = hsv[:, :, 0]
    s_values = hsv[:, :, 1]
    v_values = hsv[:, :, 2]
    cv2.imshow('single channel', v_values)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    obj_mask = []
    if color == 'green':
        lower_green = np.array([35, 70, 10])
        upper_green = np.array([85, 255, 200])
        mask_green = cv2.inRange(hsv, lower_green, upper_green)

        if adaptive:
            lower_green, upper_green = adaptive_threshold(hsv, mask_green)
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

        if filter:
            mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
        obj_mask = mask_green

        M_green = cv2.moments(mask_green)
        if M_green["m00"] > 0:
            center_point[0] = int(M_green["m10"] / M_green["m00"])
            center_point[1] = int(M_green["m01"] / M_green["m00"])

    elif color == 'red':
        lower_red1 = np.array([0, 90, 70])
        upper_red1 = np.array([10, 255, 200])
        lower_red2 = np.array([170, 90, 70])
        upper_red2 = np.array([180, 255, 200])

        mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask_red = mask_red1 | mask_red2

        if adaptive:
            lower_red1, upper_red1 = adaptive_threshold(hsv, mask_red1)
            lower_red2, upper_red2 = adaptive_threshold(hsv, mask_red2)
            mask_red1 = cv2.inRange(hsv, lower_red1, upper_red1)
            mask_red2 = cv2.inRange(hsv, lower_red2, upper_red2)
            mask_red = mask_red1 | mask_red2

        if filter:
            mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        obj_mask = mask_red
        M_red = cv2.moments(mask_red)
        if M_red["m00"] > 0:
            center_point[0] = int(M_red["m10"] / M_red["m00"])
            center_point[1] = int(M_red["m01"] / M_red["m00"])

    elif color == 'blue':
        lower_blue = np.array([100, 90, 70])
        upper_blue = np.array([140, 255, 200])
        mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        if adaptive:
            lower_blue, upper_blue = adaptive_threshold(hsv, mask_blue)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)

        if filter:
            mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)
        obj_mask = mask_blue
        M_blue = cv2.moments(mask_blue)
        if M_blue["m00"] > 0:
            center_point[0] = int(M_blue["m10"] / M_blue["m00"])
            center_point[1] = int(M_blue["m01"] / M_blue["m00"])
    else:
        raise ValueError("Wrong Color Input!")
    return center_point, obj_mask
