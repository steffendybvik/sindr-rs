alter @v1[acmag] = 1
ac dec 10 1 100k
let mag = mag(v(out))
let ph = 180/pi*cph(v(out))
wrdata @OUT@ mag ph
