$title Subscription Bus Service
OPTIONS mip = COINGLPK;
* no fixed sensors, can't stop at intermediate nodes, no budget constraints

set k vehicle /1/;
*set p passenger /1*2/;
set t time stamp /1*20/;
set i nodes /1*8/;
set i_physical(i) source node /1*6/;
set i_source(i) source node /7/;
set i_sink(i) sink node /8/;
set a acitivity /1*6/;

alias (i, j);
alias (t, s);

set w
/
0,
1
2,
1_2
/;

alias (w,wp);

parameter w_trans(w,wp) TRANSITION MATRIX/
0. 1 1
0. 2 2
1. 0 -1
1. 1_2 2
2. 0 -2
2. 1_2 1
1_2. 1 -2
1_2. 2 -1
/;

w_trans(w,w) = 1;


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
/;


*---------------------------------- transportation link--------------------------------------------------------------------------
parameter arcs(i,j,t,s) link travel time ;
*traveling arcs from/to super source and sink
arcs(i_source,i_physical,t,t)=10;
arcs(i_physical,i_sink,t,t)=10;

*traveling arcs  || flag value=10
arcs('1','2',t,t+2)=10;
arcs('2','1',t,t+2)=10;
arcs('1','3',t,t+2)=10;
arcs('3','1',t,t+2)=10;
arcs('3','4',t,t+2)=10;
arcs('4','3',t,t+2)=10;
arcs('2','4',t,t+2)=10;
arcs('4','2',t,t+2)=10;
arcs('1','5',t,t+1)=10;
arcs('2','5',t,t+1)=10;
arcs('5','2',t,t+1)=10;
arcs('5','6',t,t+1)=10;
arcs('6','3',t,t+1)=10;
arcs('6','4',t,t+1)=10;

* add ground holding arcs
arcs(i,i, t,t+1) = 10;
arcs(i_source,i_source,t,t+1)=1;
arcs(i_sink,i_sink,t,t+1)=1;


parameter travel_cost(i,j,t,s);
travel_cost(i,j,t,s)$(arcs(i,j,t,s)) = value_t(s)-value_t(t);
travel_cost(i,i,t,t+1)= 0.0001;
travel_cost(i_source,i_source,t,t+1) = 0;
travel_cost(i_sink,i_sink,t,t+1) = 0;

*---------------------------------- activity -------------------------------------------------------------------------------------

parameter act_pickup(a,i,t) the value indicating passenger delivery /
1. 1. 4  1
2. 5. 15 1
3. 3. 18 1
4. 2. 6  2
*5. 6. 9  2
*6. 4. 11 2
/;

parameter act_delivery(a,i,t) the value indicating passenger /
1. 5. 5  -1
2. 3. 17 -1
3. 1. 20 -1
4. 6. 8  -2
*5. 4. 10 -2
*6. 2. 13 -2
/;

parameter pickup(i,t) the value indicating passenger delivery /
1. 4  1
5. 15 1
3. 18 1
2. 6  2
*6. 9  2
*4. 11 2
/;
parameter delivery(i,t) the value indicating passenger /
5. 5  -1
3. 17 -1
1. 20 -1
6. 8  -2
*4. 10 -2
*2. 13 -2
/;

*---------------------------------- vehicle -------------------------------------------------------------------------------------
parameter origin_node_v(k,i,t,w)  origin nodes and departure time;
origin_node_v(k,i_source,'1','0') = 1;
parameter destination_node_v(k,i,t,w);
destination_node_v(k,i_sink,'20','0') = 1;
parameter intermediate_node_v(k,i,t,w);
intermediate_node_v(k,i,t,w) = (1- origin_node_v(k,i,t,w))*(1- destination_node_v(k,i,t,w));



*---------------------------------- transaction OD -------------------------------------------------------------------------------------

*parameter trans_arc(i,j,t,s)  transportation link flag ;
*trans_arc(i,j,t,s) = arcs(i,j,t,s);

parameter arcs_w(i,j,t,s,w,wp) link w transition ;
arcs_w(i,j,t,s,w,w)$(arcs(i,j,t,s) eq 10) = 0.01;
arcs_w(i,i,t,t,w,wp) $ ( pickup(i,t) and ( w_trans(w,wp) eq pickup(i,t))) = w_trans(w,wp);
arcs_w(i,i,t,t,w,wp) $ ( delivery(i,t) and ( w_trans(w,wp) eq delivery(i,t))) = w_trans(w,wp);

*w_trans(w,wp) =e= (p_origin(p,i,t)or p_destination(p,i,t))


*---------------------------------- equations -------------------------------------------------------------------------------------

variable z;
binary variable y(k,i,j,t,s,w,wp)  vehicle k travels between i and j from time t to time s from w to wp;

equations
obj_vehicle                                   define objective function for vehicles

v_flow_on_node_origin(k,i,t,w)                vehicle origin node flow on node i at time t
v_flow_on_node_intermediate(k,i,t,w)          vehicle intermediate node flow on node i at time t
v_flow_on_node_destination(k,i,t,w)           vehicle destination node flow on node i at time t

v_pickup_statechange(a,i,t)                 vehicle state change at the activity pickup node
v_delivery_statechange(a,j,s)               vehicle state change at the activity pickup node
;

obj_vehicle.. z =e= sum((k,i,j,t,s,w,wp)$arcs_w(i,j,t,s,w,wp), travel_cost(i,j,t,s)* y(k,i,j,t,s,w,wp));
*obj_vehicle.. z =e= sum((k,a,i,j,t,s,w,wp)$arcs_w(i,j,t,s,w,wp), (act_pickup(a,i,t)+act_delivery(a,j,s))* y(k,i,j,t,s,w,wp));
v_flow_on_node_origin(k,i,t,w)$(origin_node_v(k,i,t,w)).. sum((j,s,wp)$(arcs_w(i,j,t,s,w,wp)), y(k,i,j, t,s,w,wp)) =e= 1;
v_flow_on_node_destination(k,i,t,w)$(destination_node_v(k,i,t,w))..  sum((j,s,wp)$(arcs_w(j,i,s,t,wp,w)), y(k,j,i,s,t,wp,w))=e= 1;
v_flow_on_node_intermediate(k,i,t,w)$(intermediate_node_v(k,i,t,w)).. sum((j,s,wp)$(arcs_w(i,j,t,s,w,wp)), y(k,i,j,t,s,w,wp))-sum((j,s,wp)$(arcs_w(j,i,s,t,wp,w)), y(k,j,i,s,t,wp,w)) =e= 0;
v_pickup_statechange(a,i,t)$(act_pickup(a,i,t)).. sum((k,w,wp)$( arcs_w(i,i,t,t,w,wp) ), y(k,i,i,t,t,w,wp)*w_trans(w,wp)) =e= act_pickup(a,i,t);
v_delivery_statechange(a,j,s)$(act_delivery(a,j,s)).. sum((k,w,wp)$(arcs_w(j,j,s,s,w,wp)), y(k,j,j,s,s,w,wp)*w_trans(w,wp)) =e= act_delivery(a,j,s);

Model VPRa_vehicle /all/;


solve VPRa_vehicle using MIP minimizing z ;

display y.l;
display z.l;


File output_y; put output_y;
loop((K,T,S,I,J,W,WP)$((y.l(k,i,j,t,s,w,wp) gt 0)),put @5, k.tl, @10, i.tl, @20, j.tl,  @30, t.tl, @40, s.tl, @50, w.tl, @60, wp.tl/);
