option reltol=1e-9 vntol=1e-12 abstol=1e-15
dc v1 -1.5 5 0.1
wrdata @OUT@ v(a) v(b) v(c) i(v1) i(vdd)
