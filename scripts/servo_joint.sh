rostopic pub -r 100 /servo_server/delta_joint_cmds control_msgs/JointJog "header:
  seq: 0
  stamp: {secs: 0, nsecs: 0}
  frame_id: ''
joint_names: ['']
displacements: [0]
velocities: [0]
duration: 0.0"