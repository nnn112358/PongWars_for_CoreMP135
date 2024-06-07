CC = g++
#CFLAGS = -g -Wall -lpotrace -lm
CFLAGS = -g -Wall  -lm


SRCS = cv_main.cpp
PROG = cv_main

OPENCV = `pkg-config opencv4 --cflags --libs`
LIBS = $(OPENCV)

$(PROG):$(SRCS)
	$(CC) $(CFLAGS) -o $(PROG) $(SRCS) $(LIBS)
	

