'''
Created on 13.04.2015

@author: wypler,hrabia
''' 
from __future__ import division # force floating point division when using plain /
import rospy
import operator
import warnings
import itertools
import conditions
from std_srvs.srv import Empty, EmptyResponse
from rhbp_core.msg import Wish, Correlation, Status
from rhbp_core.srv import AddBehaviour, GetStatus, GetStatusResponse, Activate, ActivateResponse, SetInteger, SetIntegerResponse, GetPDDL, GetPDDLResponse
from .pddl import PDDL, mergeStatePDDL, create_valid_pddl_name
from utils.misc import FinalInitCaller

class Behaviour(object):
    '''
    This is the internal representation of a behaviour node
    '''

    EXECUTION_STEP_SERVICE_POSTFIX = 'ExecutionStep'
    
    _instanceCounter = 0 # static counter to get distinguishable names

    def __init__(self, name, independentFromPlanner=False, create_log_files=False, requires_execution_steps=False):
        '''
        Constructor
        '''
        self._name = name if name else "Behaviour {0}".format(Behaviour._instanceCounter)
        self._isExecuting = False   # Set this to True if this behaviour is selected for execution.
        self._correlations = []     # Stores sensor correlations as list of (sensor name <string> : correlation <float> [-1 to 1]) tuples. 1 Means high positive correlation to the value or makes it become True, -1 the opposite and 0 does not affect anything. We get this value via getStatus service of actual behaviour node
        self._wishes = []           # Stores wishes exactly like correlations. We get this via getStatus service of actual behaviour node
        self._manager = None        # This is the Manager that supplies global variables an access to all foreign objects
        self._activation = 0.0      # This is the magic activation that it's all about
        self._activationFromPreconditions = 0.0 # We get it via getStatus service of actual behaviour node
        self._preconditionSatisfaction = 0.0    # We get it via getStatus service of actual behaviour node
        self._interruptable = False # We get it via getStatus service of actual behaviour node
        self._progress = 0.0        # We get it via getStatus service of actual behaviour node
        self._readyThreshold = 0.0  # This is the threshold that the preconditionSatisfaction must reach in order for this behaviour to be executable. We get this value via getStatus service of actual behaviour node.
        self._activationDecay = 0.0 # This reduces accumulated activation if the situation does not fit any more
        self._active = True         # This indicates (if True) that there have been no severe issues in the actual behaviour node and the behaviour can be expected to be operational. If the actual behaviour reports active == False we will ignore it in activation computation.
        self._priority = 0          # The priority indicators are unsigned ints. The higher the more important
        self._manualStart = False   # If True the behaviour is started and cannot be switched off by the planner
        self._activated = True      # This member only exists as proxy for the corresponding actual behaviour's property. It is here because of the comprehensive status message published each step by the manager for rqt
        self._executionTimeout = -1 # The maximum allowed execution steps. If set to -1 infinite. We get it via getStatus service of actual behaviour node
        self._executionTime = -1    # The time the behaviour is running (in steps)
        self._reset_activation = True
        self._create_log_files = create_log_files
        self._independentFromPlanner = independentFromPlanner
        self.__currentActivationStep = 0.0
        self._justFinished = False  # This is set to True by fetchStatus if the  behaviour has just finished its job
        self.__requires_execution_steps=requires_execution_steps
        Behaviour._instanceCounter += 1
        if self._create_log_files:
            self.__logFile = open("{0}.log".format(self._name), 'w')
            self.__logFile.write('Time\t{0}\n'.format(self._name))

        if (self.__requires_execution_steps):
            rospy.wait_for_service(self._name + Behaviour.EXECUTION_STEP_SERVICE_POSTFIX)
            self.__execution_step_service = rospy.ServiceProxy(self._name + Behaviour.EXECUTION_STEP_SERVICE_POSTFIX,
                                                               Empty)
        else:
            self.__execution_step_service = None

    
    def __del__(self):
        '''
        Destructor
        '''
        self.__logFile.close()
        if (self.__execution_step_service):
            self.__execution_step_service.shutdown()

    def do_step(self):
        if not (self.__execution_step_service):
            rospy.logerr('Step method is called, but behavior does not need a step')
            return
        # notice that __execution_step_service is a callable variable of this instance and no method of class Behaviour
        self.__execution_step_service()

    def matchingCorrelations(self, effect_name):
        '''
        returns a list of correlations matching the effect_name.
        '''
        return [item[1] for item in self._correlations if item[0] == effect_name]
    
    def matchingWishes(self, effect_name):
        '''
        returns a list of indicators matching the effect_name.
        '''
        return [item[1] for item in self._wishes if item[0] == effect_name]
    
    def fetchStatus(self):
        '''
        This method fetches the status from the actual behaviour node via GetStatus service call
        '''
        self._justFinished = False
        rospy.logdebug("Waiting for service %s", self._name + 'GetStatus')
        rospy.wait_for_service(self._name + 'GetStatus')
        try:
            getStatusRequest = rospy.ServiceProxy(self._name + 'GetStatus', GetStatus)
            status = getStatusRequest().status
            self._activationFromPreconditions = status.activation
            self._correlations = [(correlation.sensorName, correlation.indicator) for correlation in status.correlations]
            self._preconditionSatisfaction = status.satisfaction
            self._readyThreshold = status.threshold
            self._wishes = [(wish.sensorName, wish.indicator) for wish in status.wishes]
            if self._isExecuting == True and status.isExecuting == False:
                rospy.loginfo("%s finished. resetting activation", self._name)
                if self._reset_activation:
                    self._activation = 0.0
                self._executionTime = -1
                self._justFinished = True
            self._isExecuting = status.isExecuting
            self._progress = status.progress
            self._active = status.active
            self._priority = status.priority
            self._interruptable = status.interruptable
            self._activated = status.activated
            self._executionTimeout = status.executionTimeout
            if self._name != status.name:
                rospy.logerr("%s fetched a status message from a different behaviour: %s. This cannot happen!", self._name, status.name)
            rospy.logdebug("%s reports the following status:\nactivation %s\ncorrelations %s\nprecondition satisfaction %s\n ready threshold %s\nwishes %s\nactive %s\npriority %d\ninterruptable %s", self._name, self._activationFromPreconditions, self._correlations, self._preconditionSatisfaction, self._readyThreshold, self._wishes, self._active, self._priority, self._interruptable)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in GetStatus of %s: %s", self._name, e)  
    
    def fetchPDDL(self):
        '''
        This method fetches the PDDL from the actual behaviour node via GetPDDLservice call.
        It returns a tuple of (action_pddl, state_pddl).
        '''
        rospy.logdebug("Waiting for service %s", self._name + 'PDDL')
        rospy.wait_for_service(self._name + 'PDDL')
        try:
            getPDDLRequest = rospy.ServiceProxy(self._name + 'PDDL', GetPDDL)
            pddl = getPDDLRequest()
            return (PDDL(statement = pddl.actionStatement, predicates = pddl.actionPredicates, functions = pddl.actionFunctions), PDDL(statement = pddl.stateStatement, predicates = pddl.statePredicates, functions = pddl.stateFunctions))
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in fetchPDDL of %s: %s", self._name, e)
            
    def getActivationFromGoals(self, goal_bias,logging = False):
        '''
        This method computes the activation from goals.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        '''
        activatedByGoals = []
        for goal in self._manager.activeGoals:
            #check for each sensor in the goal wishes for behaviours that have sensor effect correlations
            for (effect_name, indicator) in goal.wishes:
                # Make a list of all behaviours that are positively correlated to a wish of a goal (those behaviours will get activation from the goal).
                behavioursActivatedBySameGoal = [b for b in self._manager.activeBehaviours if any(map(lambda x: x * indicator > 0.0, b.matchingCorrelations(effect_name)))]
                for correlation in self.matchingCorrelations(effect_name):
                    if correlation * indicator > 0.0: # This means we affect the sensor in a way that is desirable by the goal
                        totalActivation = correlation * indicator * goal_bias
                        if logging:
                            rospy.logdebug("Calculating activation from goals for %s. There is/are %d active behaviour(s) that support(s) %s via %s: %s with a total activation of %f. GoalBias is %f",  self.name, len(behavioursActivatedBySameGoal), goal.name, effect_name, behavioursActivatedBySameGoal, totalActivation, goal_bias)
                        activatedByGoals.append((goal, totalActivation / len(behavioursActivatedBySameGoal)))        # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
        return (0.0,) if len(activatedByGoals) == 0 else (reduce(lambda x, y: x + y, (x[1] for x in activatedByGoals)), activatedByGoals)
    
    def getInhibitionFromGoals(self, conflictor_bias, logging = False):
        '''
        This method computes the inhibition (actually, activation but negative sign!) caused by goals it conflicts with.
        Precondition is that correlations are known so that the effect of this behaviour can be evaluated
        '''
        inhibitedByGoals = []
        for goal in self._manager.activeGoals:
            for (effect_name, indicator) in goal.wishes:
                # Make a list of all active behaviours that inhibit this goal.
                # Such behaviours are either negatively correlated to the goals wish (would prevent us from reaching the goal)
                # or the goal's condition has already been reached and the behaviour would undo it (goal's wish indicator is 0 but there is non-zero correlation of the behaviour to that particular sensor)
                behavioursInhibitedBySameGoal = [b for b in self._manager.activeBehaviours if any(map(lambda x: x * indicator < 0.0 or (x * indicator == 0.0 and x != 0.0), b.matchingCorrelations(effect_name)))]
                for correlation in self.matchingCorrelations(effect_name):
                    if correlation * indicator < 0.0: #This means we affect the sensor in a way that is not desirable by the goal
                        # We want the inhibition to be stronger if the condition that we would worsen is almost true.
                        # So we take -(1 - abs(indicator * correlation)) as the amount of total inhibition created by this conflict and divide it by the number of conflictors
                        totalInhibition = -(1 - abs(indicator)) * abs(correlation) * conflictor_bias
                        if logging:
                            rospy.logdebug("Calculating inhibition from goals for %s. There is/are %d behaviours(s) that worsen %s via %s: %s and a total inhibition score of %f",  self.name, len(behavioursInhibitedBySameGoal), goal.name, effect_name, behavioursInhibitedBySameGoal, totalInhibition)
                        inhibitedByGoals.append((goal, totalInhibition / len(behavioursInhibitedBySameGoal)))      # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition! Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
                    elif correlation != 0 and indicator == 0:  # That means the goals was achieved (wish indicator is 0) but we would undo that (because we are correlated to it somehow)
                        totalInhibition = -abs(correlation) * conflictor_bias
                        if logging:
                            rospy.logdebug("Calculating inhibition from goals for %s. There is/are %d behaviour(s) that undo %s via %s: %s and a total inhibition score of %f",  self.name, len(behavioursInhibitedBySameGoal), goal.name, effect_name, behavioursInhibitedBySameGoal, totalInhibition)
                        inhibitedByGoals.append((goal, totalInhibition / len(behavioursInhibitedBySameGoal)))      # The activation we get from that is the product of the correlation we have to this Sensor and the Goal's desired change of this Sensor. Note that this is negative, hence the name inhibition! Actually, we are only interested in the value itself but for debug purposed we make it a tuple including the goal itself
        return (0.0,) if len(inhibitedByGoals) == 0 else (reduce(lambda x, y: x + y, (x[1] for x in inhibitedByGoals)), inhibitedByGoals)
    
    def getActivationFromPredecessors(self,predecessor_bias, logging = False):
        '''
        This method computes the activation based on the fact that other behaviours can fulfill a precondition (wish) of this behaviour.
        This is scaled by the "readyness" (precondition satisfaction) of the predecessor as it makes only sense to activate a successor if it is likely that it is executable soon.
        '''
        activatedByPredecessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self or not behaviour.executable: # ignore ourselves and non-executable predecessors
                continue
            for (effect_name, indicator) in self._wishes: # this is what we wish from a predecessor
                # Make a list of all behaviours that share my wish (those will also get activated by the same predecessor). TODO could be improved by just counting --> less memory
                behavioursThatShareThisWish = [b for b in self._manager.activeBehaviours if any(map(lambda x: x * indicator > 0.0, b.matchingWishes(effect_name)))]
                for correlation in behaviour.matchingCorrelations(effect_name): # correlations that the behaviour (potential predecessor) has to the sensor of this wish
                    if correlation * behaviour.preconditionSatisfaction * indicator > 0.0:  # If a predecessor can satisfy my precondition
                        totalActivation = correlation * indicator * (behaviour.activation / self._manager.totalActivation) * predecessor_bias
                        if logging:
                            rospy.logdebug("Calculating activation from predecessors for %s. There is/are %d active successor(s) of %s via %s: %s with total activation of %f",  self._name, len(behavioursThatShareThisWish), behaviour.name, effect_name, behavioursThatShareThisWish, totalActivation)
                        activatedByPredecessors.append((behaviour, effect_name, totalActivation / len(behavioursThatShareThisWish))) # The activation we get is the likeliness that our predecessor fulfills the preconditions soon. behavioursThatShareThisWish knows how many more behaviours will get activation from this predecessor so we distribute it equally
        return (0.0,) if len(activatedByPredecessors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in activatedByPredecessors)), activatedByPredecessors)

    def getActivationFromSuccessors(self,successor_bias, logging = False):
        '''
        This method computes the activation this behaviour receives because it fulfills a precondition (is a predecessor) of a successor which has unfulfilled wishes.
        '''
        activatedBySuccessors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self or behaviour.executable: # ignore ourselves and successors that are already executable
                continue
            for (effect_name, indicator) in self._correlations: # this is what can give to a successor
                # Make a list of all behaviours that are correlated to the same same sensor in the same way as we are. Those are also predecessors like us an get credit from the same successor.
                behavioursThatShareOurCorrelation = [b for b in self._manager.activeBehaviours if any(map(lambda x: x * indicator > 0.0, b.matchingCorrelations(effect_name)))]
                for wish in behaviour.matchingWishes(effect_name): # if we affect other behaviour's wishes somehow
                    if wish * indicator > 0: # if we are a predecessor so we get activation from that successor
                        totalActivation = wish * indicator * (behaviour.activation / self._manager.totalActivation) * successor_bias
                        if logging:
                            rospy.logdebug("Calculating activation from successors for %s. There is/are %d active predecessor(s) of %s via %s: %s and a total activation score of %f", self._name, len(behavioursThatShareOurCorrelation), behaviour.name, effect_name, behavioursThatShareOurCorrelation, totalActivation)
                        activatedBySuccessors.append((behaviour, effect_name, totalActivation / len(behavioursThatShareOurCorrelation))) # The activation we get is our expected contribution to the fulfillment of our successors precondition. Actually only the value is needed but it is a tuple for debug purposes. len(behavioursThatShareOurCorrelation) is used to distribute activation among all predecessors
        return (0.0,) if len(activatedBySuccessors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in activatedBySuccessors)), activatedBySuccessors)

    def getInhibitionFromConflicted(self,conflictor_bias, logging = False):
        '''
        This method computes the inhibition (actually, activation but negative sign!) caused by other behaviours whose preconditions get antagonized when this behaviour runs.
        '''
        inhibitionFromConflictors = []
        for behaviour in self._manager.activeBehaviours:
            if behaviour == self: # ignore ourselves
                continue
            for (effect_name, correlation) in self._correlations: # this is what we do to sensors
                for wish in behaviour.matchingWishes(effect_name):
                    # Make a list of all behaviours that have the same bad influence on other behaviours as we have.
                    # Such behaviours are either also negatively correlated another behaviour's wish as we are
                    # or would undo an already satisfied precondition of other behaviours as we would.
                    behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation = [b for b in self._manager.activeBehaviours if any(map(lambda x: x * wish < 0.0 or (x * wish == 0.0 and x != 0.0), b.matchingCorrelations(effect_name)))]
                    if wish * correlation < 0.0:       # if we make an existing conflict stronger
                        # We want the inhibition to be stronger if the condition that we would worsen is almost true.
                        # So we take -(1 - abs(wish * correlation)) as the amount of total inhibition created by this conflict and divide it by the number of conflictors
                        totalInhibition = -(1 - abs(wish)) * abs(correlation) * (behaviour.activation / self._manager.totalActivation) * conflictor_bias
                        if logging:
                            rospy.logdebug("Calculating inhibition from conflicted for %s. %s is worsened via %s by %d behaviour(s): %s with a total score of %f", self.name, behaviour.name, effect_name, len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation), behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, totalInhibition)
                        inhibitionFromConflictors.append((behaviour, effect_name, totalInhibition / len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation))) #  Actually only the value is needed but it is a tuple for debug purposes. The length of behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation says how many more behaviours cause the same conflict to this conflictor so its inhibition shall be distributed among them.
                    elif wish * correlation == 0.0 and wish == 0:    # if we would change the currently existing good state for that behaviour (wish is zero but we have non-zero correlation to it)
                        totalInhibition = -abs(correlation) * (behaviour.activation / self._manager.totalActivation) * conflictor_bias
                        if logging:
                            rospy.logdebug("Calculating inhibition from conflicted for %s. %s is undone via %s (wish: %f) by %d behaviour(s): %s by a total score of %f", self.name, behaviour.name, effect_name, wish, len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation), behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation, totalInhibition)
                        inhibitionFromConflictors.append((behaviour, effect_name, totalInhibition / len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation))) # The inhibition experienced is my bad influence (my correlation to this effect_name) times the wish of the other behaviour concerning this effect_nameme. Actually only the value is needed but it is a tuple for debug purposes. len(behavioursThatConflictWithThatBehaviourBecauseOfTheSameCorrelation) knows how many more behaviours cause the same harm to this conflictor so its inhibition shall be distributed among them.
        return (0.0,) if len(inhibitionFromConflictors) == 0 else (reduce(lambda x, y: x + y, (x[2] for x in inhibitionFromConflictors)), inhibitionFromConflictors)

    def getActivationFromPlan(self, plan_bias, logging = False):
        '''
        This method computes the activation this behaviour receives because of its place on the plan.
        Behaviours at the top of the list will be activated most, other not so much.
        '''
        if not self._manager.plan or ("cost" in self._manager.plan and self._manager.plan["cost"] == -1.0):
            return (0.0, 0xFF)
        own_pddl_name = create_valid_pddl_name(self.name)
        for index in filter(lambda x: x >= self._manager.planExecutionIndex, sorted(self._manager.plan["actions"].keys())): # walk along the plan starting at where we are
            if self._manager.plan["actions"][index] == own_pddl_name: # if we are on the plan
                return (1 / (index - self._manager.planExecutionIndex + 1) * plan_bias, index) # index in zero-based
        return (0.0, -1)
       
    def computeActivation(self,situation_bias,plan_bias, conflictor_bias, goal_bias, successor_bias, predecessor_bias):
        '''
        This method sums up all components of activation to compute the additional activation in this step.
        @:param situation_bias: float
        '''
        #TODO this is also calculated several times, if the single functions are accessed from outside
        with_extensive_logging = False
        if self._active:
            self.__currentActivationStep = self._activationFromPreconditions *  situation_bias \
                                         + self.getActivationFromGoals(logging=with_extensive_logging, goal_bias=goal_bias)[0] \
                                         + self.getInhibitionFromGoals(logging=with_extensive_logging,conflictor_bias=conflictor_bias)[0] \
                                         + self.getActivationFromPredecessors(logging=with_extensive_logging, predecessor_bias=predecessor_bias)[0] \
                                         + self.getActivationFromSuccessors(logging=with_extensive_logging, successor_bias=successor_bias)[0] \
                                         + self.getInhibitionFromConflicted(logging=with_extensive_logging,conflictor_bias=conflictor_bias)[0] \
                                         + self.getActivationFromPlan(logging=with_extensive_logging, plan_bias=plan_bias)[0]
        else:
            return 0.0
    
    def commitActivation(self):
        '''
        This method applies the activatipn of this iteration to the overall activation.
        '''
        self._activation = self._activation * self._activationDecay + self.__currentActivationStep
        if self._activation < 0.0:
            self._activation = 0
        self.__currentActivationStep = 0.0
        if self._create_log_files:
            self.__logFile.write("{0:f}\t{1:f}\n".format(rospy.get_time(), self._activation))
            self.__logFile.flush()
    
    def start(self):
        '''
        This method calls the start service of the actual behaviour.
        It is expected that this service does not block.
        '''
        assert not self._isExecuting
        self._isExecuting = True
        self._executionTime = 0
        try:
            rospy.logdebug("Waiting for service %s", self._name + 'Start')
            rospy.wait_for_service(self._name + 'Start')
            startRequest = rospy.ServiceProxy(self._name + 'Start', Empty)
            startRequest()
            rospy.loginfo("Started action of %s", self._name)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception while calling %s: %s", self._name + 'Start', e)
    
    def stop(self, reset_activation = True):
        '''
        This method calls the stop service of the actual behaviour.
        It is expected that this service does not block.
        :param reset_activation: boolean if activation has to be reset
        '''
        assert self._isExecuting
        self._executionTime = -1
        if reset_activation:
            self.reset_activation()

        try:
            rospy.logdebug("Waiting for service %s", self._name + 'Stop')
            rospy.wait_for_service(self._name + 'Stop')
            stopRequest = rospy.ServiceProxy(self._name + 'Stop', Empty)
            stopRequest()
            rospy.loginfo("Stopping action of %s", self._name)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception while calling %s: %s", self._name + 'Stop', e)
        self._isExecuting = True # I should possibly set this at the end of try block but if that fails we are screwed anyway

    def reset_activation(self):
        self._reset_activation = True

    @property
    def requires_execution_steps(self):
        return self.__requires_execution_steps
            
    @property
    def wishes(self):
        return self._wishes
    
    @property
    def correlations(self):
        return self._correlations
    
    @property
    def activation(self):
        return self._activation
    
    @property
    def readyThreshold(self):
        return self._readyThreshold
    
    @property
    def preconditionSatisfaction(self):
        return self._preconditionSatisfaction
    
    @property
    def activationFromPreconditions(self):
        return self._activationFromPreconditions
    
    @property
    def activationDecay(self):
        return self._activationDecay
    
    @activationDecay.setter
    def activationDecay(self, rate):
        self._activationDecay = rate
        
    @property
    def manualStart(self):
        return self._manualStart
    
    @manualStart.setter
    def manualStart(self, status):
        self._manualStart = status
        
    @property
    def manager(self):
        return self._manager
    
    @manager.setter
    def manager(self, manager):
        self._manager = manager
        
    @property
    def executable(self):
        return self._preconditionSatisfaction >= self._readyThreshold
    
    @property
    def name(self):
        return self._name
    
    @property
    def active(self):
        return self._active
    
    @active.setter
    def active(self, value):
        self._active = value
    
    @property
    def activated(self):
        return self._activated
    
    @property
    def priority(self):
        return self._priority
    
    @property
    def progress(self):
        return self._progress
    
    @property
    def interruptable(self):
        return self._interruptable
    
    @property
    def isExecuting(self):
        return self._isExecuting
    
    @property
    def justFinished(self):
        return self._justFinished
    
    @property
    def independentFromPlanner(self):
        return self._independentFromPlanner
    
    @property
    def executionTimeout(self):
        return self._executionTimeout
    
    @property
    def executionTime(self):
        return self._executionTime
    
    @executionTime.setter
    def executionTime(self, value):
        self._executionTime = value
    
    def __str__(self):
        return self._name
    
    def __repr__(self):
        return self._name


class BehaviourBase(object):
    '''
    This is the base class for behaviour nodes in python
    '''

    __metaclass__ = FinalInitCaller

    def __init__(self, name, requires_execution_steps = False, **kwargs):
        '''
        Constructor
        '''
        self._name = name # a unique name is mandatory
        self._getStatusService = rospy.Service(self._name + 'GetStatus', GetStatus, self.getStatusCallback)
        self._startService = rospy.Service(self._name + 'Start', Empty, self.startCallback)
        self._stopService = rospy.Service(self._name + 'Stop', Empty, self.stopCallback)
        self._activateService = rospy.Service(self._name + 'Activate', Activate, self.activateCallback)
        self._pddlService = rospy.Service(self._name + 'PDDL', GetPDDL, self.pddlCallback)
        self._priorityService = rospy.Service(self._name + 'Priority', SetInteger, self.setPriorityCallback)
        self._executionTimeoutService = rospy.Service(self._name + 'ExecutionTimeout', SetInteger, self.setExecutionTimeoutCallback)
        self._preconditions = kwargs["preconditions"] if "preconditions" in kwargs else [] # This are the preconditions for the behaviour. They may not be used but the default implementations of computeActivation(), computeSatisfaction(), and computeWishes work them. See addPrecondition()
        self._isExecuting = False  # Set this to True if this behaviour is selected for execution.
        self._correlations = kwargs["correlations"] if "correlations" in kwargs else [] # Stores sensor correlations in list form. Expects a list of utils.Effect objects with following meaning: effect_nameme -> name of affected sensor, indicator -> value between -1 and 1 encoding how this sensor  is affected. 1 Means high positive correlation to the value or makes it become True, -1 the opposite and 0 does not affect anything. Optional condition -> a piece of pddl when this effect happens. # Be careful with the effect_name! It has to actually match something that exists!
        self._readyThreshold = kwargs["readyThreshold"] if "readyThreshold" in kwargs else 0.8 # This is the threshold that the preconditions must reach in order for this behaviour to be executable. Range [0,1]
        self._plannerPrefix = kwargs["plannerPrefix"] if "plannerPrefix" in kwargs else "" # if you have multiple planners in the same ROS environment use a prefix to name the right one.
        self._interruptable = kwargs["interruptable"] if "interruptable" in kwargs else False # The name says it all
        self._actionCost = kwargs["actionCost"] if "actionCost" in kwargs else 1.0 # This is the threshold that the preconditions must reach in order for this behaviour to be executable.
        self._priority = kwargs["priority"] if "priority" in kwargs else 0 # The priority indicators are unsigned ints. The higher the more important
        self._independentFromPlanner = kwargs["independentFromPlanner"] if "independentFromPlanner" in kwargs else False # This determines whether the manager will treat it as an error and re-plan if the behaviour run but wasn't part of the plan. Set This to true for periodic or fully reactional tasks like collision avoidance.
        self._executionTimeout = kwargs["executionTimeout"] if "executionTimeout" in kwargs else -1 # The maximum allowed execution steps. If set to -1 infinite. Interruption will only happen if interruptable flag is set (TODO: think about this again) 
        self._active = True # if anything in the behaviour is not initialized or working properly this must be set to False and communicated via getStatus service. The value of this variable is set to self._activated at the start of each status poll and should be set to False in case of errors.
        self._activated = True # The activate Service sets the value of this property.
        self._requires_execution_steps = requires_execution_steps

        if self._requires_execution_steps:
            self.__execution_step_service = rospy.Service(self._name + Behaviour.EXECUTION_STEP_SERVICE_POSTFIX, Empty,
                                                          self.do_step_callback)
        else:
            self.__execution_step_service = None


    def final_init(self):
        """
        Ensure registration after the entire initialisation (including sub classes) is done
        """
        self._register_behaviour()

    def _register_behaviour(self):
        """
        Register behaviour in the manager
        """
        try:
            rospy.logdebug("BehaviourBase constructor waiting for registration at planner manager with prefix '%s' for behaviour node %s", self._plannerPrefix, self._name)
            rospy.wait_for_service(self._plannerPrefix + 'AddBehaviour')
            registerMe = rospy.ServiceProxy(self._plannerPrefix + 'AddBehaviour', AddBehaviour)
            registerMe(self._name, self._independentFromPlanner,self._requires_execution_steps)
            rospy.logdebug("BehaviourBase constructor registered at planner manager with prefix '%s' for behaviour node %s", self._plannerPrefix, self._name)
        except rospy.ServiceException as e:
            rospy.logerr("ROS service exception in BehaviourBase constructor (for behaviour node %s): %s", self._name, e)

    def __del__(self):
        '''
        Destructor
        '''
        try:
            self._getStatusService.shutdown()
            self._startService.shutdown()
            self._stopService.shutdown()
            self._activateService.shutdown()
            self._priorityService.shutdown()
            self._executionTimeoutService.shutdown()
            self._pddlService.shutdown()
        except Exception as e:
            rospy.logerr("Error in destructor of BehaviourBase: %s", e)
            
            
    def updateComputation(self):
        """
        Updates all subentities of the behaviour in order to do computations only once
        """

        #first synchronize the input data before doing the calculation
        for p in self._preconditions:
            p.sync()
        for p in self._preconditions:
            p.updateComputation()

    def _get_satisfactions(self, include_optional=True):
        """
        :param include_optional: include optional conditions in the collection
        :returns a list filled with satisfactions of the individual preconditions
        """
        satisfactions = []  # this
        for p in filter(lambda x: x.optional == False, self._preconditions):  # check mandatory ones first because if there is an error we don't need to look at the optional ones at all
            try:
                satisfactions.append(p.satisfaction)
            except AssertionError:  # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
                self._active = False
                return 0.0
        if include_optional:
            for p in filter(lambda x: x.optional == True, self._preconditions):  # now check optional sensors
                try:
                    satisfactions.append(p.satisfaction)
                except AssertionError as e:  # we don't care about errors in optional sensors
                    rospy.logwarn(e)
                    pass
        return satisfactions
    
    def computeActivation(self):
        """
        This method returns the activation by the situation (from preconditions) as float [0 to 1]
        Note that there may be optional sensors 
        """
        activations = self._get_satisfactions(include_optional=True)

        activation_value = 1.0 if len(activations) == 0 else reduce(lambda x, y: x + y, activations) / len(activations)

        return activation_value
            
    
    def computeSatisfaction(self):
        """
        This method returns the satisfaction of the preconditions (the readiness) as float [0 to 1].
        If there are functioning optional sensors they are also handled equally like mandatory ones (so they could screw up the overall satisfaction) but if they fail they are just ignored.
        The aggregation of the satisfaction values is different to the activation calculation
        """
        satisfactions = self._get_satisfactions(include_optional=False)
        return reduce(operator.mul, satisfactions, 1)
            
    def computeWishes(self):
        """
        This method should return a list of Wish messages indicating the desired sensor changes that would satisfy its preconditions.
        A Wish message is constructed from a string (sensor name) and a desire indicator (float, [-1 to 1]).
        """
        wishes = [] # this list gets filled with the wishes of the individual preconditions
        for p in filter(lambda x: x.optional == False, self._preconditions): # check mandatory ones first because if there is an error we don't need to look at the optional ones at all
            try:
                wishes = list(itertools.chain(wishes, p.getWishes()))
            except AssertionError: # this probably comes from an uninitialized sensor or not matching activator for the sensor's data type in at least one precondition
                self._active = False
                return []
        for p in filter(lambda x: x.optional == True, self._preconditions): # now check optional sensors
            try:
                wishes = list(itertools.chain(wishes, p.getWishes()))
            except AssertionError: # we don't care about errors in optional sensors
                pass
        if self.computeSatisfaction() > self._readyThreshold: # make sure that different sensor preconditions are really disjunctive. This is not the most efficient test but it ensures that all preconditions can (or better: have) be(en) met
            wishes = self.__filterWishes(wishes)
        return [Wish(item[0], item[1]) for item in wishes]
    
    def __filterWishes(self, wishes):
        '''
        This method filters out all but the smallest wish per sensor name.
        The idea behind that is that wishes concerning the same sensor are probably disjunctive and that the behaviour is satisfied if one of the options it has are fulfilled.
        The caller must ensure that this assumption holds true, otherwise it strip away legitimate wishes without which the behaviour's preconditions can never be met.
        '''
        filteredWishes = {}
        for effect_name, indicator in wishes:
            if not effect_name in filteredWishes:
                filteredWishes[effect_name] = (effect_name, indicator)
            else:
                if abs(filteredWishes[effect_name][1]) > abs(indicator):
                    filteredWishes[effect_name] = (effect_name, indicator)
        return filteredWishes.values()
    
    def getProgress(self):
        """
        This method should return the progress of the current activities if isExecuting == True.
        It there is no current activity the value is ignored and may be filled with a dummy.
        """
        return 0.5
    
    def getActionPDDL(self):
        """
        This method should produce a valid PDDL action snippet suitable for FastDownward (http://www.fast-downward.org/PddlSupport)
        """
        action_name = create_valid_pddl_name(self._name)
        pddl = PDDL(statement =  "(:action {0}\n:parameters ()\n".format(action_name), functions = "costs")
        preconds = [x.getPreconditionPDDL(self._readyThreshold) for x in self._preconditions if not x.optional] # do not use optional preconditions for planning
        pddl.predicates = set(itertools.chain.from_iterable(map(lambda x: x.predicates, preconds))) # unites all predicates in preconditions
        pddl.functions = pddl.functions.union(*map(lambda x: x.functions, preconds)) # unites all functions in preconditions
        if len(preconds) > 1:
            pddl.statement += ":precondition (and " + " ".join(map(lambda x: x.statement, preconds)) + ")\n"
        elif len(preconds) == 1:
            pddl.statement += ":precondition " + preconds[0].statement + "\n"
        effects = [x.getEffectPDDL() for x in self._correlations]
        pddl.predicates = pddl.predicates.union(*map(lambda x: x.predicates, effects)) # adds predicates from the effects
        pddl.functions = pddl.functions.union(*map(lambda x: x.functions, effects)) # adds functions from the effects
        if len(effects) > 1:
            pddl.statement += ":effect (and (increase (costs) {0}) ".format(self._actionCost) + " ".join(map(lambda x: x.statement, effects)) + ")\n"
        elif len(effects) == 1:
            pddl.statement += ":effect (and (increase (costs) {0}) {1})\n".format(self._actionCost, effects[0].statement)
        pddl.statement += ")\n"
        return pddl
    
    def getStatePDDL(self):
        pddl = PDDL()
        for p in self._preconditions:
            if not p.optional: # do not use optional preconditions for planning
                for s in p.getStatePDDL(): # it is a list because it may come from a composed condition
                    pddl = mergeStatePDDL(s, pddl)
        return pddl                      

    def pddlCallback(self, dummy):
        if not self._independentFromPlanner and len(self._correlations) == 0:
            # Since the correlations arent setted in constructor once right place for warning is here
            rospy.logerr('Behavior {0} has no effects but is not independent from planner'.format(self._name))
        if self._independentFromPlanner and len(self._correlations) > 0:
            # Since the correlations arent setted in constructor once right place for warning is here
            rospy.logerr('Behavior {0} has effects but is independent from planner'.format(self._name))
        actions = self.getActionPDDL() # TODO: this may be cached as it does not change unless the effects are changed during runtime
        state = self.getStatePDDL()
        return GetPDDLResponse(**{"actionStatement" : actions.statement, 
                                  "actionPredicates" : list(actions.predicates),
                                  "actionFunctions" : list(actions.functions),
                                  "stateStatement" : state.statement,
                                  "statePredicates" : list(state.predicates),
                                  "stateFunctions" : list(state.functions)
                                 })
    
    def getStatusCallback(self, request):
        #update everything before generating the status message
        self.updateComputation()
        self._active = self._activated
        # TODO possible improvement is providing computeSatisfaction and computeActivation with a precalulated list of satisfactions
        # this would eliminate the doubled calculation of it
        status = Status(**{
                           "name"         : self._name, # this is sent for sanity check and planner status messages only
                           "activation"   : self.computeActivation(),
                           "correlations" : [Correlation(x.sensorName, x.indicator) for x in self._correlations],
                           "satisfaction" : self.computeSatisfaction(),
                           "threshold"    : self._readyThreshold,
                           "wishes"       : self.computeWishes(),
                           "isExecuting"  : self._isExecuting,
                           "executionTimeout" : self._executionTimeout,
                           "progress"     : self.getProgress(),
                           "active"       : self._active, # if any of the above methods failed this property has been set to False by now
                           "priority"     : self._priority,
                           "interruptable": self._is_interruptible(),
                           "activated"    : self._activated
                          })
        return GetStatusResponse(status)

    def _is_interruptible(self):
        return self._interruptable
    
    def addPrecondition(self, precondition):
        '''
        This method adds a precondition to the behaviour.
        It is not mandatory to use this method at all but it may make development easier because the default implementations of computeActivation(), computeSatisfaction(), and computeWishes work with the preconditions added here.
        If you don't want to use this mechanism then you HAVE TO implement those yourself!
        There is an AND relationship between all elements (all have to be fulfilled so that the behaviour is ready)
        To enable OR semantics use the Disjunction object.
        '''
        if issubclass(type(precondition), conditions.Conditonal):
            self._preconditions.append(precondition)
        else:
            warnings.warn("That's no conditional object!")
    
    def startCallback(self, dummy):
        '''
        This method should switch the behaviour on.
        This method must not block.
        '''
        self._isExecuting = True
        self.start()
        return EmptyResponse()
    
    def stopCallback(self, dummy):
        '''
        This method should switch the behaviour off.
        This method must not block.
        '''
        self.stop()        
        self._isExecuting = False
        return EmptyResponse()
    
    def activateCallback(self, request):
        '''
        This method activates or deactivates the behaviour.
        This method must not block.
        '''
        self._activated = request.active
        if self._activated == False:
            self.stop() 
            self._isExecuting = False
        return ActivateResponse()
    
    def setPriorityCallback(self, request):
        self._priority = request.value
        return SetIntegerResponse()
    
    def setExecutionTimeoutCallback(self, request):
        self._executionTimeout = request.value
        return SetIntegerResponse()
    
    @property
    def correlations(self):
        return self._correlations
    
    @correlations.setter
    def correlations(self, correlations):
        '''
        This is for initializing the correlations.
        correlations must be an util.Effect object.
        '''
        self._correlations = correlations
    
    @property
    def readyThreshold(self):
        return self._readyThreshold
    
    @readyThreshold.setter
    def readyThreshold(self, threshold):
        self._readyThreshold = threshold
    
    @property
    def priority(self):
        return self._priority
    
    @priority.setter
    def priority(self, priority):
        self._priority = priority
    
    @property
    def executionTimeout(self):
        return self._executionTimeout
    
    @executionTimeout.setter
    def executionTimeout(self, timeout):
        self._executionTimeout = timeout
    
    @property
    def interruptable(self):
        return self._interruptable
    
    @interruptable.setter
    def interruptable(self, interruptable):
        self._interruptable = interruptable
    
    def start(self):
        """
        This method should be overridden with one that actually does something.
        """
        pass
    
    def stop(self):
        """
        This method should be overridden with one that actually does something.
        """
        pass

    def do_step_callback(self, dummy):
        self.do_step()
        return EmptyResponse()

    def do_step(self):
        '''
        This method should be overriden, if the behavior needs steps.
        This method is called in every iteration of the manager, when the behavior is running
        '''
        pass
