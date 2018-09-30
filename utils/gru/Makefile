CC = gcc
RM = rm -f
CCFLAGS = -w

EXEEXT = 
EXECUTABLES = gru$(EXEEXT)

OBJECTS = console.o command.o info.o hw_context.o

LDLIBS = -lreadline

all: $(OBJECTS) $(EXECUTABLES)

.c.o:
	$(CC) $(CCFLAGS) -c $<	

gru$(EXEEXT): $(OBJECTS)
	$(CC) $(CCFLAGS) -o $(EXECUTABLES) $(OBJECTS) $(LDLIBS)

clean:
	$(RM) $(OBJECTS) $(EXECUTABLES)
