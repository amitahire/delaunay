SRC_DELAUNAY = delaunay.cpp debugdraw.cpp math/math.cpp sparsegrid.cpp triangulator.cpp draw.cpp cmdhelper.cpp
OBJS_DELAUNAY = $(SRC_DELAUNAY:.cpp=.o)
SRC_MAKEPOINTS = makepoints.cpp cmdhelper.cpp
OBJS_MAKEPOINTS = $(SRC_MAKEPOINTS:.cpp=.o)

.PHONY: all
all: delaunay makepoints

%.o : %.cpp
	g++ -c -o $*.o -g -Wall -ggdb -O3 -I. -DDEBUG -DLINUX $*.cpp

delaunay : $(OBJS_DELAUNAY)
	g++ -o delaunay $(OBJS_DELAUNAY) -lm -lGL -lGLU -lglut

makepoints : $(OBJS_MAKEPOINTS)
	g++ -o makepoints $(OBJS_MAKEPOINTS) -lm 

.PHONY: clean
clean:
	@rm -f $(OBJS_DELAUNAY) delaunay $(OBJS_MAKEPOINTS) makepoints

.PHONY: depend
depend:
	@rm -f .depend
	$(foreach srcfile,$(SRC_DELAUNAY),g++ -MM -I. $(srcfile) >> .depend;)
	$(foreach srcfile,$(SRC_MAKEPOINTS),g++ -MM -I. $(srcfile) >> .depend;)

-include .depend

