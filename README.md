gym_duckietown3
===

## 1 Installation (Ubuntu)

**Disclaimer: runs in Python3. Not tested in Py2.**

First pull the repo:

    git clone https://github.com/fgolemo/gym-duckietown3.git
    
then cd into that directory

    cd gym-duckietown3
    
and install (crazy, right?) in dev mode

    sudo pip3 install -e . 
    
## 2 Test

Before running any of the gym stuff you 
should have a look at the environment.
To do so, cd into the test directory...

    cd tests
    
and run the debug example

    python3 test_pybullet_roadmaker.py
    
This should launch a debug instance with sliders on the right side
where you can mess around with the motor speeds. 
If you want to zoom in, use your mouse wheel. 
If you want to rotate the debug camera, hold down CTRL and drag the mouse
in the rendering area.
You can also "throw" the robot by dragging the mouse over the red 
robot car in the big main window (I don't know if this is ever going to 
be useful for anything, but it's fun).

## 3 Gym Environments

Currently ships with 2 environments:

- `Duckiesim-StraightRoad-CPU-v0` - 3 straight road tiles, 
robot spawns on the bottom one. Duckiebot has to 
reach the center of the topmost tile.
**Reward:** given in each time step for the euclidean distance
between the robot's position and the target tile's center.
**Randomness:** Robot is spawning randomly in a circle with radius of 0.75
and center of 0,0 (= center of bottom tile). 
Robot is spawning with random orientation. Orientation starts at 90 degr 
(facing north, i.e. facing the target tile) but can be randomly rotated
in a range of \[-45,45\] degrees. 
- `Duckiesim-StraightRoad-GPU-v0` - same as above, 
but with OpenGL/GPU rendering engine. Potentially faster than CPU 
rendering. **Might** work on a cluster, 
even without XServer. But if in doubt, use CPU renderer.

In order to run these environments you can do the usual `env.make(X)` 
where X is the name of the environment as listed above. For example:

    import gym_duckietown3
    
    env = gym.make("Duckiesim-StraightRoad-CPU-v0")
    env.reset()
    
    for i in range(100):
        env.render("human")
        action = env.action_space.sample()
        obs, reward, done, misc = env.step(action)
        print(action, obs.shape, reward, done)

        time.sleep(.1)

## 4 Road Configs

TODO - but it's pretty well-documented. So in the meantime look at the file
`gym_duckietown3/road_layout.py` and at the example maps in the directory 
`gym_duckietown3/maps`.
