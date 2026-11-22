option reltol=1e-10 abstol=1e-18 vntol=1e-12
altermod dmod is=5.005e-15
op
let hi = v(k)
altermod dmod is=4.995e-15
op
let d1_is = (op1.hi - v(k)) / 1e-17
altermod dmod is=5e-15
altermod dmod n=1.6016000000000001
op
let hi = v(k)
altermod dmod n=1.5984
op
let d1_n = (op3.hi - v(k)) / 0.0032
altermod dmod n=1.6
altermod dmod rs=12.012
op
let hi = v(k)
altermod dmod rs=11.988
op
let d1_rs = (op5.hi - v(k)) / 0.024
altermod dmod rs=12.0
altermod nmod kp=0.00040040000000000003
op
let hi = v(k)
altermod nmod kp=0.0003996
op
let m1_kp = (op7.hi - v(k)) / 8.000000000000001e-07
altermod nmod kp=0.0004
altermod nmod vto=1.001
op
let hi = v(k)
altermod nmod vto=0.999
op
let m1_vto = (op9.hi - v(k)) / 0.002
altermod nmod vto=1.0
altermod nmod lambda=0.02002
op
let hi = v(k)
altermod nmod lambda=0.01998
op
let m1_lambda = (op11.hi - v(k)) / 4e-05
altermod nmod lambda=0.02
op
let d1_is = op2.d1_is
let d1_n = op4.d1_n
let d1_rs = op6.d1_rs
let m1_kp = op8.m1_kp
let m1_vto = op10.m1_vto
let m1_lambda = op12.m1_lambda
wrdata @OUT@ d1_is d1_n d1_rs m1_kp m1_vto m1_lambda
