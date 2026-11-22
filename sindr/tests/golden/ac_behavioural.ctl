option reltol=1e-9 vntol=1e-12 abstol=1e-15
ac dec 5 10 1meg
let mag = mag(v(out))
let ph = 180/pi*cph(v(out))
wrdata @OUT@ mag ph
