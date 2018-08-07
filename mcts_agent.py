#!/usr/bin/env python2
import os
import gym
import sys
import random
import itertools
from time import time
from copy import copy, deepcopy
from math import sqrt, log

import pommerman
from pommerman import agents

def moving_average(v, n):
    n = min(len(v), n)
    ret = [.0]*(len(v)-n+1)
    ret[0] = float(sum(v[:n]))/n
    for i in range(len(v)-n):
        ret[i+1] = ret[i] + float(v[n+i] - v[i])/n
    return ret


def ucb(node):
    return node.value / node.visits + sqrt(log(node.parent.visits)/node.visits)

def combinations(space):
    if isinstance(space, gym.spaces.Discrete):
        return list(range(space.n))
    elif isinstance(space, gym.spaces.Tuple):
        return itertools.product(*[combinations(s) for s in space.spaces])
    else:
        raise NotImplementedError


class Node:
    def __init__(self, parent, action):
        self.parent = parent
        self.action = action
        self.children = []
        self.explored_children = 0
        self.visits = 0
        self.value = 0

class MCTSAgent(object):
    def __init__(self, max_depth=1000, playouts=10000):
        self.max_depth = max_depth
        self.playouts = playouts 

        self.model = ForwardModel()
        self.mocked_opponents = []

    def act(self, obs, action_space):

        root = Node(None, None)
        '''
        result = self.model.step(
            actions,
            self._board,
            self._agents,
            self._bombs,
            self._items,
            self._flames,
            max_blast_strength=max_blast_strength)
        '''

        best_actions = []
        best_reward = float("-inf")

        for _ in range(self.playouts):
            # TODO: Get board, bombs, items, flames

            sum_reward = 0
            node = root
            terminal = False
            actions = []

            # selection
            while node.children:
                if node.explored_children < len(node.children):
                    child = node.children[node.explored_children]
                    node.explored_children += 1
                    node = child
                else:
                    node = max(node.children, key=ucb)

                _, reward, terminal, _ = state.step(node.action)
                sum_reward += reward
                actions.append(node.action)

            # expansion
            if not terminal:
                node.children = [Node(node, a) for a in combinations(state.action_space)]
                random.shuffle(node.children)

            # playout
            while not terminal:
                # TODO: Simulate actions
                action = state.action_space.sample()
                # TODO: Use forward-model to step
                _, reward, terminal, _ = state.step(action)

                sum_reward += reward
                actions.append(action)

                if len(actions) > self.max_depth:
                    sum_reward -= 100
                    break

            # remember best
            if best_reward < sum_reward:
                best_reward = sum_reward
                best_actions = actions

            # backpropagate
            while node:
                node.visits += 1
                node.value += sum_reward
                node = node.parent

        return best_actions[0]
