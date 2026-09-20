from enum import Enum, auto


class FlightState(Enum):
    START = auto()
    WAIT_FOR_ODOMETRY = auto()
    SEND_INITIAL_SETPOINTS = auto()
    REQUEST_OFFBOARD = auto()
    REQUEST_ARM = auto()
    TAKEOFF = auto()
    HOVER = auto()
    MISSION = auto()
    LAND = auto()
    FINISHED = auto()


class StateMachine:

    def __init__(self):

        self.state = FlightState.START

        self.counter = 0

        self.odometry_received = False

    ##########################################################

    def update_odometry(self):

        self.odometry_received = True

    ##########################################################

    def next_state(self, state):

        if self.state != state:

            print(f"\n========== {self.state.name} -> {state.name} ==========\n")

            self.state = state

            self.counter = 0

    ##########################################################

    def run(self):

        ######################################################

        if self.state == FlightState.START:

            self.next_state(
                FlightState.WAIT_FOR_ODOMETRY
            )

        ######################################################

        elif self.state == FlightState.WAIT_FOR_ODOMETRY:

            if self.odometry_received:

                self.next_state(
                    FlightState.SEND_INITIAL_SETPOINTS
                )

        ######################################################

        elif self.state == FlightState.SEND_INITIAL_SETPOINTS:

            self.counter += 1

            if self.counter > 50:

                self.next_state(
                    FlightState.REQUEST_OFFBOARD
                )

        ######################################################

        elif self.state == FlightState.REQUEST_OFFBOARD:

            self.next_state(
                FlightState.REQUEST_ARM
            )

        ######################################################

        elif self.state == FlightState.REQUEST_ARM:

            self.next_state(
                FlightState.TAKEOFF
            )

        ######################################################

        elif self.state == FlightState.TAKEOFF:

            pass

        ######################################################

        elif self.state == FlightState.HOVER:

            pass

        ######################################################

        elif self.state == FlightState.MISSION:

            pass

        ######################################################

        elif self.state == FlightState.LAND:

            pass

        ######################################################

        elif self.state == FlightState.FINISHED:

            pass