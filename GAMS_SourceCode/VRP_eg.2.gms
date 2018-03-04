* no fixed sensors, can't stop at intermediate nodes, no budget constraints



set k vehicle /1*2/;
*set p passenger /1*3/;
set p passenger /1*2/;
set t time stamp /1*30/;
set i nodes /1*12/;
set i_source(i) source node /7/;
set i_sink(i) sink node /8/;

alias (i, j);
alias (t, s);

set w
/
0,
1,
2,
*3,
1_2
*1_3,
*2_3
/;


alias (w,wp);

parameter w_trans(w,wp) TRANSITION MATRIX/
0. 1 1
0. 2 2
*0. 3 3
1. 0 -1
1. 1_2 2
*1. 1_3 3
2. 0 -2
2. 1_2 1
*2. 2_3 3
*3. 0 -3
*3. 1_3 1
*3. 2_3 2
1_2. 1 -2
1_2. 2 -1
*1_3. 1 -3
*1_3. 3 -1
*2_3. 2 -3
*2_3. 3 -2
/;

w_trans(w,w) = 0;

*==============time window====
set n time_winow_node /1*4/
parameter win(t,n) time window index /
5.1 9
6.1 9
7.1 9
16.2 10
17.2 10
18.2 10
19.2 10
8.3 11
9.3 11
10.3 11
21.4 12
22.4 12
23.4 12
24.4 12
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
10       10
11       11
12       12
/;

parameter value_p(p)/
1         1
2         2
/;
parameter arcs(i,j,t,s) link travel time ;


arcs(i_source,'4',t,t+1)=1;
arcs('1',i_sink,t,t+1)=1;
arcs('1','2',t,t+2)=1;
arcs('2','1',t,t+2)=1;
arcs('1','3',t,t+2)=1;
arcs('3','1',t,t+2)=1;
arcs('3','4',t,t+2)=1;
arcs('4','3',t,t+2)=1;
arcs('2','4',t,t+2)=1;
arcs('4','2',t,t+2)=1;
arcs('1','5',t,t+1)=1;
arcs('2','5',t,t+1)=1;
arcs('5','6',t,t+1)=1;
arcs('6','3',t,t+1)=1;
arcs('6','4',t,t+1)=1;
arcs('2','9',t,t+1)=1;
arcs('5','10',t,t+1)=1;
arcs('9','2',t,t+1)$(win(t,'1')= 9)=1;
arcs('10','5',t,t+1)$(win(t,'2')= 10)=1;
arcs('6','11',t,t+1)=1;
arcs('11','6',t,t+1)$(win(t,'3')= 11)=1;
arcs('3','12',t,t+1)=1;
arcs('12','3',t,t+1)$(win(t,'4')= 12)=1;

parameter trans_arcs(i,j,t,s)  transportation link flag ;
trans_arcs(i,j,t,s) =arcs(i,j,t,s)  ;

* add ground holding arcs
parameter waiting_arcs(i,j,t,s) ;
waiting_arcs(i,i, t,t+1) = 1;

parameter travel_cost(k,i,j,t,s);

travel_cost(k,i,j,t,s)$(trans_arcs(i,j,t,s)) = value_t(s)-value_t(t);
travel_cost(k,i,j,t,s)$(waiting_arcs(i,j,t,s))= 0.5;



parameter origin_node(k,i,t,w)  origin nodes and departure time;
origin_node(k,i_source,'1','0') = 1;

parameter destination_node(k,i,t,w);
destination_node(k,i_sink,'30','0') = 1;

parameter intermediate_node(k,i,t,w);
intermediate_node(k,i,t,w) = (1- origin_node(k,i,t,w))*(1- destination_node(k,i,t,w));



parameter p_origin(i,t);
p_origin('9',t)$(win(t,'1')= 9)=1;
p_origin('10',t)$(win(t,'2')= 10)=2;

parameter p_destination(i,t);
p_destination('11',t)$(win(t,'3')= 11)=-1;
p_destination('12',t)$(win(t,'4')= 12)=-2;




parameter arcs_w(i,j,t,s,w,wp) link w transition ;


arcs_w(i,j,t,s,w,w)$(trans_arcs(i,j,t,s)) =1;
arcs_w(i,j,t,s,w,w)$(waiting_arcs(i,j,t,s)) =1;
arcs_w(i,j,t,s,w,wp)$(p_origin(i,t)=1 and (w_trans(w,wp) eq p_origin(i,t))and trans_arcs(i,j,t,s))= 1;
arcs_w(i,j,t,s,w,wp)$(p_origin(i,t)=2 and (w_trans(w,wp) eq p_origin(i,t))and trans_arcs(i,j,t,s))= 1;
arcs_w(i,j,t,s,w,wp)$(p_destination(i,t)= -1 and (w_trans(w,wp) eq p_destination(i,t))and trans_arcs(i,j,t,s))= 1;
arcs_w(i,j,t,s,w,wp)$(p_destination(i,t)= -2 and (w_trans(w,wp) eq p_destination(i,t))and trans_arcs(i,j,t,s))= 1;




parameter arcs_pickup(p,i,j,t,s,w,wp);
arcs_pickup(p,i,j,t,s,w,wp)$((p_origin(i,t)=value_p(p)) and (w_trans(w,wp) eq p_origin(i,t)) and arcs(i,j,t,s))= 1;

parameter arcs_delivery(p,i,j,t,s,w,wp);
arcs_delivery(p,i,j,t,s,w,wp)$((p_destination(i,t)=-value_p(p)) and (w_trans(w,wp) eq p_destination(i,t)) and arcs(i,j,t,s))= 1;


variable z;
binary variable y(k,i,j,t,s,w,wp)  vehicle k travels between i and j from time t to time s from w to wp;

equations
obj                                   define objective function

flow_on_node_origin(k,i,t,w)         origin node flow on node i at time t
flow_on_node_intermediate(k,i,t,w)   intermediate node flow on node i at time t
flow_on_node_destination(k,i,t,w)      destination node flow on node i at time t
pickup_flow_on_each_passenger(p)
delivery_flow_on_each_passenger(p)
;

obj.. z =e= sum((k,i,j,t,s,w,wp)$arcs_w(i,j,t,s,w,wp), travel_cost(k,i,j,t,s)* y(k,i,j,t,s,w,wp));
flow_on_node_origin(k,i,t,w)$(origin_node(k,i,t,w)).. sum((j,s,wp)$(arcs_w(i,j,t,s,w,wp)), y(k,i,j, t,s,w,wp)) =e= 1;
flow_on_node_destination(k,i,t,w)$(destination_node(k,i,t,w))..  sum((j,s,wp)$(arcs_w(j,i,s,t,w,wp)), y(k,j,i,s,t,w,wp))=e= 1;
flow_on_node_intermediate(k,i,t,w)$(intermediate_node(k,i,t,w)).. sum((j,s,wp)$(arcs_w(i,j,t,s,w,wp)), y(k,i,j,t,s,w,wp))-sum((j,s,wp)$(arcs_w(j,i,s,t,wp,w)), y(k,j,i,s,t,wp,w)) =e= 0;
pickup_flow_on_each_passenger(p)$(value_p(p))..sum((k,i,j,t,s,w,wp)$(arcs_pickup(p,i,j,t,s,w,wp)), y(k,i,j, t,s,w,wp)) =e= 1;
delivery_flow_on_each_passenger(p)$(value_p(p))..sum((k,i,j,t,s,w,wp)$(arcs_delivery(p,i,j,t,s,w,wp)), y(k,i,j, t,s,w,wp)) =e= 1;

Model customized_bueses_optimization /ALL/;
solve customized_bueses_optimization using MIP  minimizing z ;
display y.l;
display z.l;

File output_process/output_eg2.csv/;
put output_process;

put @5,'z,'/
put @5,z.l,',' /;

put @5,'k,',@15,'i,',@25,'j,',@35,'t,',@45,'s,',@55,'w,',@65,'wp,' ,@75,'y,'/

loop((k,i,j,t,s,w,wp)$((y.l(k,i,j,t,s,w,wp) = 1)),put@5,k.tl,',',@15,i.tl,',',@25,j.tl,',',@35,t.tl,',',@45, s.tl,',' ,@55,w.tl,',',@65,wp.tl,',',@75, y.l(k,i,j,t,s,w,wp)/);

