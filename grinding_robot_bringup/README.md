## UR launch documentation
<arg name="robot_ip" default="192.168.56.42" doc="IP address by which the robot can be reached." />
  <arg name="reverse_ip" default=""
    doc="IP of the driver, if set to empty it will detect it automatically." />
  <arg name="reverse_port" default="50001"
    doc="Port that will be opened by the driver to allow direct communication between the driver and the robot controller." />
  <arg name="script_sender_port" default="50002"
    doc="The driver will offer an interface to receive the program's URScript on this port. If the robot cannot connect to this port, `External Control` will stop immediately." />
  <arg name="trajectory_port" default="50003"
    doc="Port that will be opened by the driver to allow trajectory forwarding." />
  <arg name="script_command_port" default="50004"
    doc="Port that will be opened by the driver to allow forwarding script commands to the robot." />
  <arg name="tf_prefix" default="" doc="tf_prefix used for the robot." />
  <arg name="kinematics_config"
    default="$(find grinding_descriptions)/config/ur5e/default_kinematics.yaml"
    doc="Kinematics config file used for calibration correction. This will be used to verify the robot's calibration is matching the robot_description." />
  <arg name="use_tool_communication" default="false"
    doc="On e-Series robots tool communication can be enabled with this argument" />
  <arg name="tool_voltage" default="0"
    doc="Tool voltage set at the beginning of the UR program. Only used, when `use_tool_communication` is set to true." />
  <arg name="tool_parity" default="0"
    doc="Parity configuration used for tool communication. Only used, when `use_tool_communication` is set to true." />
  <arg name="tool_baud_rate" default="115200"
    doc="Baud rate used for tool communication. Only used, when `use_tool_communication` is set to true." />
  <arg name="tool_stop_bits" default="1"
    doc="Number of stop bits used for tool communication. Only used, when `use_tool_communication` is set to true." />
  <arg name="tool_rx_idle_chars" default="1.5"
    doc="Number of idle chars in RX channel used for tool communication. Only used, when `use_tool_communication` is set to true." />
  <arg name="tool_tx_idle_chars" default="3.5"
    doc="Number of idle chars in TX channel used for tool communication. Only used, when `use_tool_communication` is set to true." />
  <arg name="tool_device_name" default="/tmp/ttyUR"
    doc="Local device name used for tool communication. Only used, when `use_tool_communication` is set to true." />
  <arg name="tool_tcp_port" default="54321"
    doc="Port on which the robot controller publishes the tool comm interface. Only used, when `use_tool_communication` is set to true." />
  <arg name="headless_mode" default="false"
    doc="Automatically send URScript to robot to execute. On e-Series this does require the robot to be in 'remote-control' mode. With this, the URCap is not needed on the robot." />
  <arg name="ur_hardware_interface_node_required" default="true"
    doc="Shut down ros environment if ur_hardware_interface-node dies." />
  
  <!--Robot description and related parameter files -->
  <arg name="robot_description_file" default="$(find grinding_descriptions)/launch/load_ur5e.launch" doc="Robot description launch file." />
  <!-- <arg name="robot_description_file" default="$(dirname)/inc/load_ur5e.launch.xml" doc="Launch file which populates the 'robot_description' parameter."/> -->
  <arg name="joint_limit_params"
    default="$(find grinding_descriptions)/config/ur5e/joint_limits.yaml" />
  <arg name="kinematics_params" default="$(arg kinematics_config)" />
  <arg name="physical_params"
    default="$(find grinding_descriptions)/config/ur5e/physical_parameters.yaml" />
  <arg name="visual_params"
    default="$(find grinding_descriptions)/config/ur5e/visual_parameters.yaml" />

  <!-- Controller configuration -->
  <arg name="controller_config_file" default="$(find ur_gazebo)/config/ur5e_controllers.yaml" doc="Config file used for defining the ROS-Control controllers."/>
  <arg name="controllers" default="joint_state_controller eff_joint_traj_controller" doc="Controllers that are activated by default."/>
  <arg name="stopped_controllers" default="joint_group_eff_controller" doc="Controllers that are initally loaded, but not started."/>

  <!-- robot_state_publisher configuration -->
  <arg name="tf_prefix" default="" doc="tf_prefix used for the robot."/>
  <arg name="tf_pub_rate" default="500" doc="Rate at which robot_state_publisher should publish transforms."/>

  <!-- Gazebo parameters -->
  <arg name="paused" default="false" doc="Starts Gazebo in paused mode" />
  <arg name="gui" default="true" doc="Starts Gazebo gui" />


