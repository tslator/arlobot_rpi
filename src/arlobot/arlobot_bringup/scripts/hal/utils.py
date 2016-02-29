#!/usr/bin/env python
from __future__ import print_function

import time
from threading import Thread, Event, Lock


class Worker (Thread):
    def __init__(self, name, work):
        Thread.__init__(self)
        self._stop = Event()
        self._name = name
        self._work = work
    def run(self):
        print("{} thread started".format(self._name))
        while not self._stop.isSet():
            self._work()
        print("{} thread stopped".format(self._name))
    def stop(self):
        self._stop.set()


