This is a tiny implementation of printf that includes floating point format,
but has much less memory footprint than the implementation from newlib-nano
(linked with "-u \_printf_float" switch).

I had to modify it somehow, to truly override standard print functions from
standard library AND not to cause problems with the #define printf printf_
overriding my functions that have 'printf' in name.

Solution:
changed the defines form ("direction") in *printf.h* from this:
```
#define printf printf_
```
to this:
```
#define printf_ printf
```
