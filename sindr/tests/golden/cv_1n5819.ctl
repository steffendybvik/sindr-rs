option reltol=1e-9 vntol=1e-12 abstol=1e-15
save v(b) @d1[cd] @d1[gd]
dc v1 -10 0.2 0.1
wrdata @OUT@ v(b) @d1[cd] @d1[gd]
