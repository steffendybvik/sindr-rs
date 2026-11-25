option reltol=1e-6
tran 10u 10m 0 1u uic
linearize
wrdata @OUT@ v(out)
