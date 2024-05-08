World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:box, Q:[0 0.05 -.05], size:[1.35 2.5 .05 .02], color:[.3 .3 .3]
    contact:1, logical:{ }
}

table_left (table_base){
    shape:box, Q:[-1.2 0.05 -.05], size:[0.75 2.5 .05 .02], color:[.3 .3 .3]
    contact:1, logical:{ }
}

table_right (table_base){
    shape:box, Q:[1.2 0.05 -.05], size:[0.75 2.5 .05 .02], color:[.3 .3 .3]
    contact:1, logical:{ }
}


