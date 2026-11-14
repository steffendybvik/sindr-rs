option reltol=1e-6
tran 20u 5m 0 1u uic
linearize
wrdata @OUT@ v(out)
