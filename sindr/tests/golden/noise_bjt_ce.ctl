option reltol=1e-9 vntol=1e-12 abstol=1e-15
alter @vin[acmag] = 1
noise v(c) vin dec 5 10 100k
setplot noise1
wrdata @OUT@ onoise_spectrum inoise_spectrum
