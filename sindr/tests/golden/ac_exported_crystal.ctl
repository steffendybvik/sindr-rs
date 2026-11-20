alter @v1[acmag] = 1
ac dec 200 1meg 100meg
let mag = mag(v(out))
let ph = 180/pi*cph(v(out))
wrdata @OUT@ mag ph
