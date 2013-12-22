SDIO Stuff
----------

This is some code I am working on for reading SD Cards. This
is the 'simple' code (it runs in polled mode). 

Ok, so updated this repo to be the complete example set. The
initial part was a memory explorer which I have morphed into 
an SD card explorer as well. Note that if you're running this
it will be most fun to have two terminal sessions going to your
board, one for the debug stream and one for the console stream.
(stderr and stdout if you will :-). 

Note that the example itself is kind of useless, and its built
on my machine inside the libopencm3-examples tree ( under
libopencm3-examples/stm32/f4/stm32f4-discovery/xplor ) if you're
having trouble building it drop me an email. Also tool chain is
the launchpad.net arm-none-eabi- toolchain from ARM inc. Guys
you totally rock! And the Black Magic Debug Probe has been
invaluable in getting this going, Gareth you're my hero. 

All open source code development for embedded systems is the best!

--Chuck

