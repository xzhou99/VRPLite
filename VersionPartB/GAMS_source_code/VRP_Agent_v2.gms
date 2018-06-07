$title Subscription Bus Service
OPTIONS mip = COINGLPK;
* no fixed sensors, can't stop at intermediate nodes, no budget constraints

set k vehicle /1/;
set p passenger /1*2/;
set t time stamp /1*30/;
set i nodes /1*9/;
set i_source(i) source node /7/;
set i_sink(i) sink node /8/;
set a activity /1*8/;


alias (i, j);
alias (t, s);
alias (a, ap);


parameter value_p(p)/
1        1
2        2
/;

parameter value_t(t)/
1        1
2        2
3        3
4        4
5        5
6        6
7        7
8        8
9        9
10        10
11        11
12        12
13        13
14        14
15        15
16        16
17        17
18        18
19        19
20        20
21        21
22        22
23        23
24        24
25        25
26        26
27        27
28        28
29        29
30        30
/;

parameter value_i(i)/
1        1
2        2
3        3
4        4
5        5
6        6
7        7
8        8
9        9
/;


*---------------------------------- transportation link--------------------------------------------------------------------------
parameter arcs(i,j,t,s) link travel time ;

*traveling arcs  || flag value=10
arcs('7','1',t,t+1)=10;
arcs('1','7',t,t+1)=10;
arcs('7','2',t,t+1)=10;
arcs('2','7',t,t+1)=10;
arcs('1','2',t,t+1)=10;
arcs('2','1',t,t+1)=10;
arcs('1','3',t,t+5)=10;
arcs('3','1',t,t+5)=10;
arcs('1','6',t,t+5)=10;
arcs('6','1',t,t+5)=10;
arcs('2','6',t,t+5)=10;
arcs('6','2',t,t+5)=10;
arcs('2','5',t,t+5)=10;
arcs('5','2',t,t+5)=10;
arcs('3','6',t,t+1)=10;
arcs('6','3',t,t+1)=10;
arcs('3','8',t,t+1)=10;
arcs('8','3',t,t+1)=10;
arcs('3','4',t,t+1)=10;
arcs('4','3',t,t+1)=10;
arcs('6','4',t,t+1)=10;
arcs('4','6',t,t+1)=10;
arcs('6','5',t,t+1)=10;
arcs('5','6',t,t+1)=10;
arcs('4','8',t,t+1)=10;
arcs('8','4',t,t+1)=10;
arcs('4','5',t,t+1)=10;
arcs('5','4',t,t+1)=10;
arcs('3','9',t,t)=10;
arcs('9','3',t,t)=10;

* add ground holding arcs
arcs(i,i, t,t+1) = 10;
arcs(i_source,i_source,t,t+1)=1;
arcs(i_sink,i_sink,t,t+1)=1;

parameter travel_cost(i,j,t,s);
travel_cost(i,j,t,s)$(arcs(i,j,t,s)) = value_t(s)-value_t(t);
travel_cost(i,i,t,t+1)= 0.0001;
travel_cost(i_source,i_source,t,t+1) = 0;
travel_cost(i_sink,i_sink,t,t+1) = 0;




*---------------------------------- trip request -------------------------------------------------------------------------------------
* trip request occurs at node i with time window

parameter trip_pickup(a,i,t) the value indicating passenger pickup /
1. 1. 3  1
2. 3. 15 1
3. 6. 21 1
4. 2. 2  2
6. 9. 15 2
5. 4. 17 2
8. 5. 20 2
/;

parameter trip_delivery(a,i,t) the value indicating passenger delivery/
2. 3. 10  -1
3. 6. 16  -1
1. 1. 27  -1
5. 4. 9   -2
6. 9. 10  -2
8. 5. 18  -2
4. 2. 26  -2
/;


parameter pickup(i,t) the value indicating passenger pickup /
1. 3  1
3. 15 1
6. 21 1
2. 2  2
9. 15 2
4. 17 2
5. 20 2
/;

parameter delivery(i,t) the value indicating delivery /
3. 10  -1
6. 16  -1
1. 27  -1
4. 9   -2
9. 10  -2
5. 18  -2
2. 26  -2
/;



* what set w includes is the trip request tau other than people
set w
/
0,
1
2,
2_2,
1_2,
1_2_2
/;

alias (w,wp);

parameter w_trans(w,wp) TRANSITION MATRIX/
0. 1             1
0. 2             2
0. 2_2           2
1. 0             -1
1. 1_2           2
1. 1_2_2         2
2. 0             -2
2_2. 0           -2
2. 1_2           1
2_2. 1_2_2       1
1_2. 1           -2
1_2_2. 1         -2
1_2. 2           -1
1_2_2. 2_2       -1
2. 2_2            2
2_2. 2            -2
1_2. 1_2_2        2
1_2_2. 1_2        -2
/;

w_trans(w,w) = 0;

*---------------------------------- transaction OD -------------------------------------------------------------------------------------

*parameter trans_arc(i,j,t,s)  transportation link flag ;
*trans_arc(i,j,t,s) = arcs(i,j,t,s);

parameter arcs_w(i,j,t,s,w,wp) link w transition ;
arcs_w(i,j,t,s,w,w)$(arcs(i,j,t,s) eq 10) = 0.01;
arcs_w(i,i,t,t,w,wp) $ ( pickup(i,t) and ( w_trans(w,wp) eq pickup(i,t))) = w_trans(w,wp);
arcs_w(i,i,t,t,w,wp) $ ( delivery(i,t) and ( w_trans(w,wp) eq delivery(i,t))) = w_trans(w,wp);

File output_yy; put output_yy;
loop((I,T,W,WP)$((arcs_w(i,i,t,t,w,wp))),put @10, i.tl, @20, i.tl,  @30, t.tl, @40, t.tl, @50, w.tl, @60, wp.tl/);



*---------------------------------- vehicle -------------------------------------------------------------------------------------
parameter origin_node_v(k,i,t,w)  origin nodes and departure time;
origin_node_v(k,'7','1','0') = 1;
parameter destination_node_v(k,i,t,w);
destination_node_v(k,'7','30','0') = 1;
parameter intermediate_node_v(k,i,t,w);
intermediate_node_v(k,i,t,w) = (1- origin_node_v(k,i,t,w))*(1- destination_node_v(k,i,t,w));


*---------------------------------- pax  -------------------------------------------------------------------------------------
parameter arc_pas(p,a,ap) feasible activity connection/
1. 1. 2  1
1. 2. 3  1
1. 2. 1  1
1. 3. 1  1
2. 4. 5  1
2. 5. 6  1
2. 6. 4  1
2. 6. 5  1
2. 5. 4  1
2. 5. 8  1
2. 8. 4  1
/;

parameter origin_node_p(p,a);
origin_node_p('1','1') = 1;
origin_node_p('2','4') = 1;
parameter destination_node_p(p,a);
destination_node_p('1','1') = 1;
destination_node_p('2','4') = 1;
parameter intermediate_node_p(p,a);
intermediate_node_p(p,a) = (1- origin_node_p(p,a))*(1- destination_node_p(p,a));

parameter utility(p,a,ap);
utility(p,a,ap)$ (arc_pas(p,a,ap))=1;

*---------------------------------- equations -------------------------------------------------------------------------------------

variable z;
binary variable y(k,i,j,t,s,w,wp)  vehicle k travels between i and j from time t to time s from w to wp;
binary variable x(p,a,ap)  passenger p travels from activity a to ap;


equations
*obj_vehicle                                   define objective function for vehicles
obj_passenger                                 define objective function for passenger
v_flow_on_node_origin(k,i,t,w)                vehicle origin node flow on node i at time t
v_flow_on_node_intermediate(k,i,t,w)          vehicle intermediate node flow on node i at time t
v_flow_on_node_destination(k,i,t,w)           vehicle destination node flow on node i at time t

v_pickup_statechange(a,i,t)                 vehicle state change at the activity pickup node
v_delivery_statechange(a,j,s)               vehicle state change at the activity pickup node


p_flow_on_node_origin(p,a)                pax activity origin node flow
p_flow_on_node_destination(p,a)           pax activity destination node
p_flow_on_node_intermediate(p,a)          pax activity intermediate node

;

*obj_vehicle.. z =e= sum((k,i,j,t,s,w,wp)$arcs_w(i,j,t,s,w,wp), travel_cost(i,j,t,s)* y(k,i,j,t,s,w,wp));
obj_passenger..z =e= sum((p,a,ap)$arc_pas(p,a,ap), utility(p,a,ap)* x(p,a,ap));
v_flow_on_node_origin(k,i,t,w)$(origin_node_v(k,i,t,w)).. sum((j,s,wp)$(arcs_w(i,j,t,s,w,wp)), y(k,i,j, t,s,w,wp)) =e= 1;
v_flow_on_node_destination(k,i,t,w)$(destination_node_v(k,i,t,w))..  sum((j,s,wp)$(arcs_w(j,i,s,t,wp,w)), y(k,j,i,s,t,wp,w))=e= 1;
v_flow_on_node_intermediate(k,i,t,w)$(intermediate_node_v(k,i,t,w)).. sum((j,s,wp)$(arcs_w(i,j,t,s,w,wp)), y(k,i,j,t,s,w,wp))-sum((j,s,wp)$(arcs_w(j,i,s,t,wp,w)), y(k,j,i,s,t,wp,w)) =e= 0;

v_pickup_statechange(a,i,t)$(trip_pickup(a,i,t)).. sum((k,w,wp)$(arcs_w(i,i,t,t,w,wp) > 0.01  ), y(k,i,i,t,t,w,wp)*w_trans(w,wp)) =e= trip_pickup(a,i,t);
v_delivery_statechange(a,j,s)$(trip_delivery(a,j,s)).. sum((k,w,wp)$(arcs_w(j,j,s,s,w,wp) < 0), y(k,j,j,s,s,w,wp)*w_trans(w,wp)) =e= trip_delivery(a,j,s);

p_flow_on_node_origin(p,a)$(origin_node_p(p,a)).. sum((ap)$(arc_pas(p,a,ap)), x(p,a,ap)) =e= 1;
p_flow_on_node_destination(p,a)$(destination_node_p(p,a))..  sum((ap)$(arc_pas(p,ap,a)), x(p,ap,a))=e= 1;
p_flow_on_node_intermediate(p,a)$(intermediate_node_p(p,a)).. sum((ap)$(arc_pas(p,a,ap)), x(p,a,ap))-sum((ap)$(arc_pas(p,ap,a)), x(p,ap,a)) =e= 0;


Model VPRa_vehicle /all/;


solve VPRa_vehicle using MIP maximizing z ;

display x.l;
display y.l;
display z.l;


File output_y; put output_y;
loop((K,T,S,I,J,W,WP)$((y.l(k,i,j,t,s,w,wp) )),put @5, k.tl, @10, i.tl, @20, j.tl,  @30, t.tl, @40, s.tl, @50, w.tl, @60, wp.tl, @70, y.l(k,i,j,t,s,w,wp)/);
File output_x; put output_x;
loop((P,A,AP)$((x.l(p,a,ap) gt 0)),put @5, p.tl, @10, a.tl, @20, ap.tl/);
