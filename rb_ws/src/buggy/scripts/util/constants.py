import math

class Constants:
    UTM_EAST_ZERO = 589702.87
    UTM_NORTH_ZERO = 4477172.947
    UTM_ZONE_NUM = 17
    UTM_ZONE_LETTER = "T"
    WHEELBASE_SC = 1.104
    WHEELBASE_NAND = 1.3

    # https://en.wikipedia.org/wiki/Circular_error_probable
    CEP50_to_STD = 1 / math.sqrt(-2 * math.log(0.5)) #0.8493218003

    OFFSET_THRESHOLD = ((1 * 3) * math.pi/180)**2 # Convert 1 deg std dev to 3sigma variance (rad)