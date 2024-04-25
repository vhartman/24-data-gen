World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

Include: '../../../rai-robotModels/husky/husky.g'
Edit base_link {X:<[0, -1, 0.15, 0.707, 0, 0, 0.707]>}

#Prefix: "a0_"
#Include: '../robots/ur5.g'
#Edit a0_base (left_arm_bulkhead_joint) { Q:[0 0 0.] }

#Prefix: "a1_"
#Include: '../robots/ur5.g'
#Edit a1_base (right_arm_bulkhead_joint) { Q:[0 0 .05] }

table_base (World) {
    Q:[0 0 .7]
    shape:marker, size:[.03],
}
table (table_base){
    shape:box, Q:[0 0.05 -.05], size:[2.3 1.44 .05 .02], color:[.3 .3 .3]
    contact, logical:{ }
}


