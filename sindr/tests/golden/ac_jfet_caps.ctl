option reltol=1e-9 vntol=1e-12 abstol=1e-15
alter @vin[acmag] = 1
ac dec 5 10k 1g
let mag = mag(v(d))
let ph = 180/pi*cph(v(d))
wrdata @OUT@ mag ph
