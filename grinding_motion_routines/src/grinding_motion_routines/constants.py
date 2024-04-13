import rospy

TRAC_IK = "trac_ik"
CARTESIAN_COMPLIANCE_CONTROLLER = "cartesian_compliance_controller"
JOINT_SUBSCRIBER = "/arm_controller/state"
JOINT_STATE_SUBSCRIBER = "joint_states"
FT_SUBSCRIBER = "wrench"

BASE_LINK = "base_link"
EE_LINK = "tool0"
FT_LINK = "tool0"

# RESULT_CODE
DONE = "done"
FORCE_TORQUE_EXCEEDED = "force_exceeded"
STOP_ON_TARGET_FORCE = "stop_on_target_force"
IK_NOT_FOUND = "ik_not_found"
SPEED_LIMIT_EXCEEDED = "speed_limit_exceeded"
TERMINATION_CRITERIA = "termination_criteria_achieved"
