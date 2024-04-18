Include: '../../../rai-robotModels/kuka_drake/kuka_no_box.g'

## zero position
Edit iiwa_joint_1 { q= 0.0 }
Edit iiwa_joint_2 { q= -0.5 }
Edit iiwa_joint_3 { q= 0.0 }
Edit iiwa_joint_4 { q= -1.5 }
Edit iiwa_joint_5 { q= -3.0 }
Edit iiwa_joint_6 { q= 0.2 }
Edit iiwa_joint_7 { q= 0.0 }

#gripper (panda_joint7){
#    shape:sphere, size:[.03]
#    Q:<t(.11 0.0 .0)>
#    contact:0 
#    color:[1.0,0.5,0.5,1.0]
#}

# add robotiq
Include: '../../../rai-robotModels/robotiq/robotiq.g'
Edit robotiq_base (iiwa_joint_7) { Q:[0 0 .05] }

# filling the hole
#gripper_fill (iiwa_joint_7){ shape:cylinder, color:[.1, .1, .1 ,1], size:[.1 .031]}

# pen
pen_tip (robotiq_base){ 
    shape:sphere,
    color:[.9, 0, 0 ,1], 
    Q:<t(.0 0.0 .15)>,
    size:[0.005],
    contact:0
}

pen_tip_marker (pen_tip){
    shape: marker,
    size:[0.05]
}

#gripper (panda_joint7){
#    shape:sphere, size:[.02]
#    Q:<t(.24 0.0 .0)>
#    contact:0 
#    color:[1.0,0.5,0.5,1.0]
#}

#gripper (panda_joint7){
#    shape:sphere, size:[.02]
#    Q:<t(.125 0.0 .0)>
#    contact:0 
#    color:[1.0,0.5,0.5,1.0]
#}

#gripper_p1 (panda_joint7){
#    shape:sphere, size:[.02]
#    Q:<t(0.125 -0.03 0.01)>
#    contact:0
#    color:[1.0,0.5,0.5,1.0]
#}
#
#gripper_p2 (panda_joint7){
#    shape:sphere, size:[.02]
#    Q:<t(0.125 0.03 0.01)>
#    contact:0
#    color:[1.0,0.5,0.5,1.0]
#}
#
#gripper_p3 (panda_joint7){
#    shape:sphere, size:[.02]
#    Q:<t(0.125 -0.0 -0.04)>
#    contact:0
#    color:[1.0,0.5,0.5,1.0]
#}
#
#gripper_vis (panda_joint7){
#    shape:cylinder, size:[.04 0.05]
#    Q:<t(0.125 0.0 0.00) d(90 0 1 0)>
#    contact:0
#    color:[1.0,0.5,0.5,.5]
#}
#
#gripper_coll (panda_joint7){
#    shape:cylinder, size:[.03 0.05]
#    Q:<t(0.125 0.0 0.0) d(90 0 1 0)>
#    contact:0
#    color:[1.0,0.5,0.5,.5]
#}
