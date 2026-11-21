option reltol=1e-6
run
linearize
wrdata @OUT@ v(out) v(s)
