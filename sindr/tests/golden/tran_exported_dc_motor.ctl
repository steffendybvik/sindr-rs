option reltol=1e-6
tran 20u 30m 0 10u uic
linearize
wrdata @OUT@ v(m1_w) i(v1)
