# Lets specify compiler
CC = gcc
CFLAGS = -c -Wall

# Source files.  We keep header files separate from the cpp files.
# Recall that we will compile cpp files, but we do not compile
# the header files.
HEADER = PID.h

SOURCES = Motor.c PID.c

# Program name
PROGNAME = Motor

# Object files
OBJ = $(SOURCES:.c=.o)

all: $(PROGNAME)

$(PROGNAME): $(OBJ)
	$(CC) -o $(PROGNAME) $(OBJ)

%.o: %.c $(HEADER)
	$(CC) $(CFLAGS) -o $@ $<

# Target clean is phony, since it doesn't
# create a file called clean.  Notice that other
# targets create files.
.PHONY: clean

# We will use target clean to delete both the object files
# and the program.
clean:
	rm $(PROGNAME) $(OBJ)
