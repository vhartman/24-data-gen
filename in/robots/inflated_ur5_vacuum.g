Include: './ur5e_inflated.g'

# filling the hole
gripper_fill (wrist_3_joint){ shape:capsule, color:[.1, .1, .1 ,.1], size:[.17 .06], Q:<t(.0 0.0 0.07)>, contact:-3}
vacuum_coll (wrist_3_joint){ shape:cylinder, color:[.1, .1, .1 ,1], size:[.07 .01], Q:<t(.0 0.0 0.17)>, contact:-3}
camera_coll (wrist_3_joint){ shape:capsule, color:[.1, .1, .1 ,0.1], size:[.07 .04], Q:<t(-.09 0.0 0.08) d(110 0 1 0)>, contact:-3}

gripper_coupling (wrist_3_joint){ shape:cylinder, color:[.1, .1, .1 , 1], size:[.016 .037], Q:<t(.0 0.0 0.06)>, contact: 0}
gripper (wrist_3_joint){ shape:cylinder, color:[.1, .1, .1 ,0.5], size:[.12 .037], Q:<t(.0 0.0 0.125)>, contact:-3}

# pen
pen_tip (wrist_3_joint){ 
    shape:sphere,
    color:[.9, 0, 0 ,1], 
    Q:<t(.0 0.0 0.23)>,
    size:[0.005],
    contact:0
}

pen_tip_marker (pen_tip){
    shape: marker,
    size:[0.05]
}