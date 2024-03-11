# OFDM_TxRx

Simple application of what I learned about the OFDM signals technique, here for posterity and backing up.  
Lets you encode a byte stream (no more than 65536 long, though not tested above 2000 or so) in packets of 160 (NFFT with cylic prefix) samples, with some headers and footers to help.  
Resistance to noise is nil. Possible improvement path.

## Usage

Simple API demonstrated in `source/main.c`, the logic itself would benefit from more comments... probably later

## License

BSD 3-clause for compatibility with bitstream and kissfft

## Special thanks

Many StackOverflow posts I lost the links to over time, notably for the cross_correlation function!
