#!/usr/bin/env python
from openravepy import *
RaveInitialize()
RaveLoadPlugin('build/gasoto')
try:
    env=Environment()
    env.Load('scenes/myscene.env.xml')
    rrt = RaveCreateModule(env,'rrt')
    print rrt.SendCommand('help')
finally:
    RaveDestroy()
