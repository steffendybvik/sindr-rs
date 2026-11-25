option reltol=1e-5
tran 5n 6m 0 5n uic
meas tran vout avg v(out) from=5.9m to=6m
meas tran iin avg i(vin) from=5.9m to=6m
meas tran iripple pp i(l1) from=5.9m to=6m
meas tran vbefore avg v(out) from=5.8m to=5.9m
setplot new
let vout = tran1.vout
let iin = tran1.iin
let iripple = tran1.iripple
let vbefore = tran1.vbefore
wrdata @OUT@ vout iin iripple vbefore
