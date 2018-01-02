class RoadMap(object):
    ## Resolution of the road config
    ## format: rows, cols
    road_rez = (4, 3)

    ## Where should the robot spawn? I.e. which tile should be 0,0
    ## format: row, col
    road_start = (2,0)

    ## TILES:
    ## 0 - no tile
    ## 1 - straight (default orientation: facing north / same as robot)
    ## 2 - turn (default orientation: turn north-west)

    ## ORIENTATIONS:
    ## 0 - vanilla
    ## 1 - 90 degr right
    ## 2 - 180 degr
    ## 3 - 90 degr left

    # road_conf = [
    #     [(1, 0), (0, 0), (0, 0)],
    #     [(1, 0), (0, 0), (0, 0)],
    #     [(1, 0), (0, 0), (0, 0)],
    #     [(1, 0), (0, 0), (0, 0)]
    # ]
    road_conf = [
        [(2, 1), (1, 1), (2, 0)],
        [(1, 0), (0, 0), (1, 0)],
        [(1, 0), (0, 0), (1, 0)],
        [(2, 2), (1, 1), (2, 3)]
    ]
