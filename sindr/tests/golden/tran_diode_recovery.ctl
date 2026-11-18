option reltol=1e-6
tran 0.1n 300n 0 0.1n uic
linearize
wrdata @OUT@ v(b) i(v1)
