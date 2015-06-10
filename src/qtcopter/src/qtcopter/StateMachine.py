from . import Userdata
import rospy


class StateMachine:
    FINAL_STATE = '__finished__'

    def __init__(self, states, transitions, start, outcomes):
        if start not in states:
            raise RuntimeError('Invalid start state {0}'.format(start))
        self.__states = states
        self.__transitions = transitions
        self.__outcomes = outcomes
        self.__start = start
        self.current_state = start

    def __call__(self, userdata):
        if self.current_state == self.FINAL_STATE:
            raise RuntimeError('State machine finished!')

        output = Userdata()
        outcome = self.__states[self.current_state](userdata, output)
        if outcome not in self.__states[self.current_state].outcomes:
            raise RuntimeError('Invalid outcome {0}.'.format(outcome))
        if outcome in self.__transitions:
            next_state = self.__transitions[outcome]
            if (next_state != self.current_state):
                rospy.loginfo('Transition from "{0}" to "{1}".'
                              .format(self.current_state, next_state))
            self.current_state = next_state
        else:
            assert(outcome in self.__outcomes)
            self.current_state = self.FINAL_STATE
            rospy.loginfo('State machine finished.')
        return output

    def is_finished(self):
        return self.current_state == self.FINAL_STATE

    def write_dot(self, stream):
        stream.write('digraph {')
        # Write start state.
        stream.write('S [shape=point];')
        stream.write('S -> "{0}";'.format(self.__start))
        # Write final state.
        stream.write('"Finished" [shape=doublecircle];')
        # Write intermediate nodes/edges.
        state_outcomes = [(outcome, state) for state in self.__states
                          for outcome in self.__states[state].outcomes]
        for outcome, state in state_outcomes:
            if outcome in self.__transitions:
                stream.write('"{0}" -> "{1}" [label="{2}"];'.format(state, self.__transitions[outcome], outcome))
            else:
                assert(outcome in self.__outcomes)
                stream.write('"{0}" -> "{1}" [label="{2}"];'.format(state, 'Finished', outcome))
        stream.write('}')
