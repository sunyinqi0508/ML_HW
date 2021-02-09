# valueIterationAgents.py
# -----------------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


# valueIterationAgents.py
# -----------------------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


import mdp, util

from learningAgents import ValueEstimationAgent
import collections

class ValueIterationAgent(ValueEstimationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A ValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100):
        """
          Your value iteration agent should take an mdp on
          construction, run the indicated number of iterations
          and then act according to the resulting policy.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state, action, nextState)
              mdp.isTerminal(state)
        """
        self.mdp = mdp
        self.discount = discount
        self.iterations = iterations
        self.values = util.Counter() # A Counter is a dict with default 0
        self.runValueIteration()

    def runValueIteration(self):
        # Write value iteration code here
        "*** YOUR CODE HERE ***"
        from copy import deepcopy
        states = self.mdp.getStates()
        for i in range(0, self.iterations):
            newval = deepcopy(self.values)
            for st in states:
                val_ = -float('inf')
                actions = self.mdp.getPossibleActions(st)
                for ac in actions:
                    qval = self.computeQValueFromValues(st, ac)
                    if(qval > val_):
                        val_ = qval
                if val_ == -float('inf'):
                    val_ = 0
                newval[st] = val_
            self.values = newval
    def getValue(self, state):
        """
          Return the value of the state (computed in __init__).
        """
        return self.values[state]


    def computeQValueFromValues(self, state, action):
        """
          Compute the Q-value of action in state from the
          value function stored in self.values.
        """
        "*** YOUR CODE HERE ***"
        q = 0
        for state_, prob in self.mdp.getTransitionStatesAndProbs(state, action):
            q += prob * (self.mdp.getReward(state, action, state_) + 
                    self.discount * self.getValue(state_))
        return q

    def computeActionFromValues(self, state):
        """
          The policy is the best action in the given state
          according to the values currently stored in self.values.

          You may break ties any way you see fit.  Note that if
          there are no legal actions, which is the case at the
          terminal state, you should return None.
        """
        "*** YOUR CODE HERE ***"
        actions = self.mdp.getPossibleActions(state)
        if len(actions) == 0:
            return None
        val_ = -float('inf')
        action_ = actions[0]
        for ac in actions:
            qval = self.computeQValueFromValues(state, ac)
            if(qval > val_):
                val_ = qval
                action_ = ac
        return action_

    def getPolicy(self, state):
        return self.computeActionFromValues(state)

    def getAction(self, state):
        "Returns the policy at the state (no exploration)."
        return self.computeActionFromValues(state)

    def getQValue(self, state, action):
        return self.computeQValueFromValues(state, action)

class AsynchronousValueIterationAgent(ValueIterationAgent):
    """
        * Please read learningAgents.py before reading this.*

        An AsynchronousValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs cyclic value iteration
        for a given number of iterations using the supplied
        discount factor.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 1000):
        """
          Your cyclic value iteration agent should take an mdp on
          construction, run the indicated number of iterations,
          and then act according to the resulting policy. Each iteration
          updates the value of only one state, which cycles through
          the states list. If the chosen state is terminal, nothing
          happens in that iteration.

          Some useful mdp methods you will use:
              mdp.getStates()
              mdp.getPossibleActions(state)
              mdp.getTransitionStatesAndProbs(state, action)
              mdp.getReward(state)
              mdp.isTerminal(state)
        """
        ValueIterationAgent.__init__(self, mdp, discount, iterations)

    def runValueIteration(self):
        "*** YOUR CODE HERE ***"
        i = 0
        states = self.mdp.getStates()
        while i < self.iterations:
            for st in states:
                if i >= self.iterations:
                    break
                val_ = -float('inf')
                actions = self.mdp.getPossibleActions(st)
                for ac in actions:
                    qval = self.computeQValueFromValues(st, ac)
                    if(qval > val_):
                        val_ = qval
                if val_ == -float('inf'):
                    val_ = 0
                self.values[st] = val_
                i += 1

class PrioritizedSweepingValueIterationAgent(AsynchronousValueIterationAgent):
    """
        * Please read learningAgents.py before reading this.*

        A PrioritizedSweepingValueIterationAgent takes a Markov decision process
        (see mdp.py) on initialization and runs prioritized sweeping value iteration
        for a given number of iterations using the supplied parameters.
    """
    def __init__(self, mdp, discount = 0.9, iterations = 100, theta = 1e-5):
        """
          Your prioritized sweeping value iteration agent should take an mdp on
          construction, run the indicated number of iterations,
          and then act according to the resulting policy.
        """
        self.theta = theta
        ValueIterationAgent.__init__(self, mdp, discount, iterations)

    def runValueIteration(self):
        "*** YOUR CODE HERE ***"
        pred = {}
        states = self.mdp.getStates()
        for state in states:
            pred[state] = set()
        for state in states:
            actions = self.mdp.getPossibleActions(state)
            for action in actions:
                for state_, prob in self.mdp.getTransitionStatesAndProbs(state, action):
                    if prob > 0: 
                        pred[state_].add(state)
        q = util.PriorityQueue()
        for state in states:
            actions = self.mdp.getPossibleActions(state)
            val_ = -float('inf')
            for action in actions:
                qval = self.computeQValueFromValues(state, action)
                if(val_ < qval):
                    val_ = qval
            if val_ == -float('inf'):
                pass
            else:
                q.push(state, -abs(self.getValue(state) - val_))
        i = 0
        while(not q.isEmpty() and i < self.iterations):
            state = q.pop()
            actions = self.mdp.getPossibleActions(state)
            val_ = -float('inf')
            for ac in actions:
                qval = self.computeQValueFromValues(state, ac)
                if(qval > val_):
                    val_ = qval
            if val_ == -float('inf'):
                val_ = 0
            self.values[state] = val_
            for st in pred[state]:
                actions = self.mdp.getPossibleActions(st)
                qval = -float('inf')
                for action in actions:
                    qval = max(qval, self.computeQValueFromValues(st, action))
                if qval == -float('inf'):
                    pass
                else:       
                    diff = abs(self.getValue(st) - qval)
                    if diff > self.theta:
                        q.update(st, -diff)
            i += 1