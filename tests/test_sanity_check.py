import gym
import gym_duckietown3  # mandatory for next line
import time

env = gym.make("Duckiesim-StraightRoad-CPU-v0")

env.reset()
print("### GOING FORWARD / [1,1]")
for i in range(100):
    env.render("human")
    action = [1, 1]
    env.step(action)

env.reset()
print("### GOING BACKWARD / [0,0]")
for i in range(100):
    env.render("human")
    action = [0, 0]
    env.step(action)

env.reset()
print("### TURNING LEFT / [0,1]")
for i in range(100):
    env.render("human")
    action = [0, 1]
    env.step(action)

env.reset()
print("### TURNING RIGHT / [1,0]")
for i in range(100):
    env.render("human")
    action = [1, 0]
    env.step(action)

env.reset()
print("### THIS SHOULDN'T WORK: BACKSPIN / [-100,-100]")
for i in range(100):
    env.render("human")
    action = [-100, -100]
    env.step(action)
