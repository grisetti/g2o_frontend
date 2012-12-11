OBJS = 	pwn_cloud.o \
	pwn_solve.o \
	pwn_normals.o

CPPFLAGS = -I/usr/include/eigen3
CXXFLAGS = -O3

all: $(OBJS)

.phony:	clean all


%.o : %.cpp %.h
	$(CXX) -o $@ -c $(CPPFLAGS) $(CXXFLAGS) $<

clean:
	rm -f *~ *.o