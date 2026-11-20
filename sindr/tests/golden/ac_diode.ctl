option reltol=1e-9 vntol=1e-12 abstol=1e-15
alter @v1[acmag] = 1
ac dec 5 1k 1g
let mag = mag(v(a))
let ph = 180/pi*cph(v(a))
wrdata @OUT@ mag ph
