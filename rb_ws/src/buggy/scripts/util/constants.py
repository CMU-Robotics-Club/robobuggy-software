import math
import numpy as np

class Constants:
    UTM_EAST_ZERO = 589702.87
    UTM_NORTH_ZERO = 4477172.947
    UTM_ZONE_NUM = 17
    UTM_ZONE_LETTER = "T"
    WHEELBASE_SC = 1.104
    WHEELBASE_NAND = 1.3

    T_LIDAR_TO_CAMERA = np.array([
        [-1.0,  0.0,  0.0, -0.046  ],
        [ 0.0, -1.0,  0.0,  0.0    ],
        [ 0.0,  0.0,  1.0, -0.05625],
        [ 0.0,  0.0,  0.0,  1.0    ]
    ])
    