#!/usr/bin/env python

"""
The module contains helper methods related to logging
"""

from rospy import loginfo, logdebug, logwarn, logerr, logfatal

def rospylog(level, text):
    """
    :description: Translate log call with level to appropriate rospy log* call.  The method permits a level to be pass to
    a common log method.
    :param level: One of 'info', 'debug', 'warn', 'err', or 'fatal'
    :param text: The text to be output
    :return: None
    """
    if level == 'info':
        loginfo(text)
    elif level == 'debug':
        logdebug(text)
    elif level == 'warn':
        logwarn(text)
    elif level == 'err':
        logwarn(text)
    elif level == 'fatal':
        logfatal(text)
    else:
        logfatal("Unknown level: {} - {}".format(level, text))
