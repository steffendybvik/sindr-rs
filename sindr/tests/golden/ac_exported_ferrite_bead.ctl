alter @v1[acmag] = 1
ac dec 20 100k 10g
let mag = mag(v(out))
let ph = 180/pi*cph(v(out))
wrdata @OUT@ mag ph
