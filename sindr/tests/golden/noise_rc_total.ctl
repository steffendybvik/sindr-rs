alter @v1[acmag] = 1
noise v(out) v1 dec 10 1 100k
setplot noise2
let idx = vector(1)
wrdata @OUT@ idx onoise_total inoise_total
