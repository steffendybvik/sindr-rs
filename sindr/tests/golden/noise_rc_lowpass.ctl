alter @v1[acmag] = 1
noise v(out) v1 dec 10 1 100k
setplot noise1
wrdata @OUT@ onoise_spectrum inoise_spectrum
