World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:ssBox, Q:[0 0.05 -.05], size:[2.3 1.44 .05 .02], color:[.3 .3 .3]
    contact, logical:{ }
}

_table_spacer (table) 	{  type:ssBox, size:[0.5 0.5 0.01 .005], contact:1 Q:<[  0.0, 0, 0.05, 1, 0, .0, .0]> color:[0, 0, 0, 0.01]}

_obs (World) 	{  type:ssBox, size:[3 0.2 1 .005], contact:1 Q:<[  0.0, -0.8, 1.1, 1, 0, .0, .0]> color:[0, 0, 0, 0.1]}

