#!/usr/bin/env python

import numpy as np


class Circular_Queue:

    def __init__(self, length):
        self.length = length
        self.tail = length - 1
        self.queue = np.empty(length, dtype=np.uint8)

    def enqueue(self, datum):
        self.tail = (self.tail + 1) % self.length
        self.queue[self.tail] = datum
    
    def peek(self):
        return self.queue[(self.tail + 1) % self.length]

    def at(self, index):
        return self.queue[(self.tail + 1 + index) % self.length]

    def average(self):
        return np.average(self.queue)