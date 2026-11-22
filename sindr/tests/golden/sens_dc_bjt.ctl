option reltol=1e-10 abstol=1e-18 vntol=1e-12
altermod qmod bf=150.15
op
let hi = v(c)
altermod qmod bf=149.85
op
let q1_bf = (op1.hi - v(c)) / 0.3
altermod qmod bf=150.0
altermod qmod is=2.002e-15
op
let hi = v(c)
altermod qmod is=1.998e-15
op
let q1_is = (op3.hi - v(c)) / 4e-18
altermod qmod is=2e-15
altermod qmod br=3.003
op
let hi = v(c)
altermod qmod br=2.997
op
let q1_br = (op5.hi - v(c)) / 0.006
altermod qmod br=3.0
altermod qmod nf=1.02102
op
let hi = v(c)
altermod qmod nf=1.01898
op
let q1_nf = (op7.hi - v(c)) / 0.00204
altermod qmod nf=1.02
altermod qmod nr=1.1011000000000002
op
let hi = v(c)
altermod qmod nr=1.0989
op
let q1_nr = (op9.hi - v(c)) / 0.0022
altermod qmod nr=1.1
op
let q1_bf = op2.q1_bf
let q1_is = op4.q1_is
let q1_br = op6.q1_br
let q1_nf = op8.q1_nf
let q1_nr = op10.q1_nr
wrdata @OUT@ q1_bf q1_is q1_br q1_nf q1_nr
