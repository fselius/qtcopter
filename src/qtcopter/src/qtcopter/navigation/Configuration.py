#!/usr/bin/env python

# Really simple configuration class
# Constructor with file path, defaults to config.json if not passed
# Throws exception if file not found
# Loads parsed json data to self.data
# Can get configuration section by GetConfigurationSection(section)
# Which returns the section or None if no such section

import os.path
import json

class DictWatch(dict):
    def __init__(self, *args):
        dict.__init__(self, args)

    def __getitem__(self, key):
        val = dict.__getitem__(self, key)
        return val

    def __setitem__(self, key, val):
        dict.__setitem__(self, key, val)

class Configuration:
    def __init__(self, path=None):
        self.path='config.json'
        if path is not None:
            if os.path.isfile(path):
                self.path=path
            else:
                raise IOError("File not found: {0}".format(path))
        with open(self.path) as config:
            self.data = json.load(config)

    def GetConfigurationSection(self, section):
        if section in self.data:
            return self.data[section]
        return None

    def UpdateConfiguration(self):
        if os.path.isfile(self.path):
            with open(self.path) as config:
                self.data = json.load(config)
        else:
            raise IOError("Configuration file not found: {0}, Cannot update configs".format(self.path))


    def SetNewConfigFile(self, path):
        oldPath = self.path
        if os.path.isfile(path):
            try:
                self.path=path
                self.UpdateConfiguration()
            except IOError as e:
                self.path = path
                raise e


conf = Configuration('NavConfig.json')
#
# Usage example:
# Create Configuration object with NavConfig.json file
# Get 'TakeOff' section returns all section
# Get 'NotFoundSection' returns None
# takeoffParams is a dictionary
# try:
#     conf = Configuration('NavConfig.json')
#     takeoffParams = conf.GetConfigurationSection('Takeoff')
#     print(takeoffParams['Throttle'])
#     print(conf.GetConfigurationSection('NotFoundSection'))
#     example = conf.GetConfigurationSection('example')
#     for d in example['Missionslist']:
#         print("Mission: {0}".format(d))
#     print(example['Boolean'])
# except IOError as e:
#     print(e.message)