class HardwareAbstractionLayer(object):
    __shared_state = {}

    __HAL_STATE_INITIAL = "INITIAL"
    __HAL_STATE_STOPPED = "STOPPED"
    __HAL_STATE_RUNNING = "RUNNING"
    __hal_state = __HAL_STATE_INITIAL
    __lock = Lock()
    __counter = 0

    def __init__(self, use_stub=True):
        self.__dict__ = self.__shared_state
        self._use_stub = use_stub
        rospy.loginfo("HAL state: {}".format(HardwareAbstractionLayer.__hal_state))
        rospy.loginfo("HAL Counter: {}".format(HardwareAbstractionLayer.__counter))
        HardwareAbstractionLayer.__counter += 1
        print("HAL __dict__ {}", self.__dict__)

    def _initialize(self):
        # Do all the object initialization here
        self._newstuff = 0
        pass

    def Startup(self):
        rospy.loginfo("HAL startup invoked")
        with HardwareAbstractionLayer.__lock:
            current_state = HardwareAbstractionLayer.__hal_state

        rospy.loginfo("HAL state: {}".format(HardwareAbstractionLayer.__hal_state))
        if current_state is HardwareAbstractionLayer.__HAL_STATE_RUNNING:
            rospy.logwarn("Attempt to startup HAL while it is running - No action taken")
            return

        # Handle case when state is INITIAL - the shared object state needs to be setup
        if current_state is HardwareAbstractionLayer.__HAL_STATE_INITIAL:
            rospy.loginfo("HAL state: {}".format(HardwareAbstractionLayer.__hal_state))
            self._initialize()
            rospy.loginfo("Initialize Complete")

            with HardwareAbstractionLayer.__lock:
                HardwareAbstractionLayer.__hal_state = HardwareAbstractionLayer.__HAL_STATE_STOPPED
                current_state = HardwareAbstractionLayer.__hal_state
            rospy.loginfo("HAL state: {}".format(HardwareAbstractionLayer.__hal_state))

        # Handle case when the shared object state is initialize and ready for the next level of startup
        if current_state is HardwareAbstractionLayer.__HAL_STATE_STOPPED:
            rospy.loginfo("HAL performing addition startup ...")

            # Do other initialization/startup here, like starting the wheel threads when using stubs

            rospy.loginfo("Complete")

            with HardwareAbstractionLayer.__lock:
                HardwareAbstractionLayer.__hal_state = HardwareAbstractionLayer.__HAL_STATE_RUNNING
                current_state = HardwareAbstractionLayer.__hal_state
            rospy.loginfo("HAL state: {}".format(HardwareAbstractionLayer.__hal_state))

            if hasattr(HardwareAbstractionLayer, "__callbacks"):
                rospy.loginfo("HAL calling HALConnect callbacks")
                for callback in HardwareAbstractionLayer.__callbacks:
                    callback()
                rospy.loginfo("Complete")

        rospy.loginfo("HAL state: {}".format(HardwareAbstractionLayer.__hal_state))
        rospy.loginfo("Leaving Startup")

    def Connect(self, callback):
        rospy.loginfo("HAL connect invoked")
        rospy.logwarn("HAL __dict__ {}, {}".format(self.__dict__, HardwareAbstractionLayer.__hal_state))
        with HardwareAbstractionLayer.__lock:
            current_state = HardwareAbstractionLayer.__hal_state

        if current_state is HardwareAbstractionLayer.__HAL_STATE_RUNNING:
            rospy.loginfo("HAL state: {}".format(HardwareAbstractionLayer.__hal_state))
            rospy.loginfo("Immediately calling HALConnect callback to release condition")
            callback()
        else:
            rospy.loginfo("HAL state: {}".format(HardwareAbstractionLayer.__hal_state))
            try:
                rospy.loginfo("Attempting to add callback to list")
                HardwareAbstractionLayer.__callbacks.append(callback)
            except AttributeError:
                rospy.loginfo("No callback list exists, creating list and attempting to add callback")
                HardwareAbstractionLayer.__callbacks = []
                HardwareAbstractionLayer.__callbacks.append(callback)
        rospy.loginfo("HAL connect complete")

    def Shutdown(self):
        rospy.loginfo("HAL shutdown invoked")
        while HardwareAbstractionLayer.__lock:
            current_state = HardwareAbstractionLayer.__hal_state

        rospy.loginfo("HAL is current in {} state".format(current_state))

        # Is there anything we need to do here?  What about sending a command to the Psoc4 to powerdown the HB25's

    def Ready(self):
        with HardwareAbstractionLayer.__lock:
            ready = HardwareAbstractionLayer.__hal_state == "RUNNING"

        return ready
