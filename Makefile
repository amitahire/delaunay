SRC = delaunay.cpp debugdraw.cpp math/math.cpp sparsegrid.cpp triangulator.cpp draw.cpp
OBJS = $(SRC:.cpp=.o)

%.o : %.cpp
	g++ -c -o $*.o -g -Wall -ggdb -O3 -I. -DDEBUG -DLINUX $*.cpp

delaunay : $(OBJS)
	g++ -o delaunay $(OBJS) -lm -lGL -lGLU -lglut

.PHONY: clean
clean:
	@rm -f $(OBJS) delaunay

.PHONY: depend
depend:
	@rm -f .depend
	$(foreach srcfile,$(SRC),g++ -MM -I. $(srcfile) >> .depend;)

-include .depend

