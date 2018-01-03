from gym_duckietown3.road_layout import RoadLayout


class MapStraightLine3x1(RoadLayout):
    map_rez = (3, 1)
    map_start = (2, 0)  # bottom tile
    map_conf = [
        [(1, 0)],
        [(1, 0)],
        [(1, 0)]
    ]
