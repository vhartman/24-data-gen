World 	{  X:<[0, 0, 0, 1, 0, 0, 0]> }                         

table_base (World) {
    Q:[0 0 .6]
    shape:marker, size:[.03],
}
table (table_base){
    shape:box, Q:[0 0.05 -.05], size:[3 2.5 .05 .02], color:[.3 .3 .3]
    contact:1, logical:{ }
}
