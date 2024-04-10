Include: '../../../rai-robotModels/ur5/ur5e.g'

# add robotiq
#Include: '../../rai-robotModels/robotiq/robotiq.g'
#Edit robotiq_base (wrist_3_joint) { Q:[0 0 .05] }

# filling the hole
gripper_fill (wrist_3_joint){ shape:cylinder, color:[.1, .1, .1 ,1], size:[.17 .021], contact:-1}

# pen
pen_tip (wrist_3_joint){ 
    shape:sphere,
    color:[.9, 0, 0 ,1], 
    Q:<t(.0 0.0 0.135)>,
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
