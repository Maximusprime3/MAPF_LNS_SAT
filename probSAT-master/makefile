CC=gcc
CFLAGS= -Wall -Wextra -static -O3 -funroll-loops -fexpensive-optimizations -D_POSIX_C_SOURCE=199309L
#CFLAGS=  -pg -ggdb -Wall -lm   -Wno-missing-braces -static 

all: probSAT

probSAT:	probSAT.c
			$(CC) $(CFLAGS)  probSAT.c -lm -o probSAT
clean:	
		rm -f probSAT

