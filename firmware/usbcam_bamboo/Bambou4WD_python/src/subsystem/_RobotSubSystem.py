'''
Created on 22 janv. 2018

@author: OLIVIERCousinier
'''

class _RobotSubSystem(object):
    def __init__(self, phmi, pcomRobotCommand):
        self.hmi=phmi
        self.comCommand=pcomRobotCommand
        return