#!/usr/bin/env python
"""Run ros from root of project and load config variables"""
import json
with open("projectConfig.json", "r") as jsonfile:
    configData = json.load(jsonfile)
    print("Read config successful")

worldPath =  configData["worldpath"]
height = configData["height"]
width = configData["width"]


print("     path to world is: " + worldPath + "\n\n")

print("     area is " +  str(width*height))
