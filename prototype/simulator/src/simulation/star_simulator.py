"""
本文件将生成模拟星图，包含多颗星星，不同星等，并添加噪声和PSF效应
PSF是点扩散函数，Point spread function，
理想情况下，一个点光源经过成像系统应该形成完美的点，即艾里斑，
但是实际光学系统存在相差，衍射，大气湍流和噪声，
导致光源会扩散成一个有一定形状和大小模糊的斑点
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.ndimage import gaussian_filter
from dataclasses import dataclass
from typing import List, Tuple, Optional
import json
import os

@dataclass
class Star:
    ra_deg:float #赤经
    dec_deg:float #赤纬
    magnitude:float #星等
    spectral_type:str  = "G2V" #光谱类型选择太阳类型
