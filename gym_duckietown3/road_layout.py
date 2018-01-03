class RoadLayout(object):
    map_rez = (2, 2)
    """ Resolution of the map
        I.e. how many rows and columns of tiles should there be
        format: rows, cols
    """

    map_start = (1, 0)
    """ Where should the robot spawn? 
        I.e. which tile should be 0,0
        format: row, col
    """

    map_conf = [
        [(2, 1), (1, 1)],  # right turn, straight road 90 deg right
        [(1, 0), (0, 0)],  # straight road (start), empty tile
    ]
    """ Actual road layout
        format:
        
        TILES:
            0 - no tile
            1 - straight (default orientation: facing north / same as robot)
            2 - turn (default orientation: turn north-west)
    
        ORIENTATIONS:
            0 - vanilla
            1 - 90 degr right
            2 - 180 degr
            3 - 90 degr left
    """


    ## example with a straight line
    # map_conf = [
    #     [(1, 0), (0, 0), (0, 0)],
    #     [(1, 0), (0, 0), (0, 0)],
    #     [(1, 0), (0, 0), (0, 0)],
    #     [(1, 0), (0, 0), (0, 0)]
    # ]

    ## example with a full loop
    # map_conf = [
    #     [(2, 1), (1, 1), (2, 0)],
    #     [(1, 0), (0, 0), (1, 0)],
    #     [(1, 0), (0, 0), (1, 0)],
    #     [(2, 2), (1, 1), (2, 3)]
    # ]
