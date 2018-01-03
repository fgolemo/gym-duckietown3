from gym_duckietown3.road_layout import RoadLayout


class MapFullLoop4x3(RoadLayout):
    map_rez = (4, 3)
    map_start = (2, 0)
    map_conf = [
        [(2, 1), (1, 1), (2, 0)],
        [(1, 0), (0, 0), (1, 0)],
        [(1, 0), (0, 0), (1, 0)],
        [(2, 2), (1, 1), (2, 3)]
    ]
