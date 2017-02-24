close all; clear all;
obj = LookupTableGenerator();
[t1, t2, t3, t4, t5, t6] = ndgrid(40:50, 40:50,40:50,40:50,40:50,40:50);

f = @(t1, t2, t3, t4, t5, t6) obj.configure([t1 t2 t3 t4 t5 t6]);
V = f(t1, t2, t3, t4, t5, t6);

f = @(t1, t2, t3, t4, t5, t6) obj.configure([t1 t2 t3 t4 t5 t6]);
// while true
// 	obj.refresh();
// end
