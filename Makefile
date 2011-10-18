SRC_DELAUNAY = delaunay.cpp debugdraw.cpp math/math.cpp sparsegrid.cpp triangulator.cpp draw.cpp cmdhelper.cpp
OBJS_DELAUNAY = $(SRC_DELAUNAY:.cpp=.o)
SRC_MAKEPOINTS = makepoints.cpp cmdhelper.cpp
OBJS_MAKEPOINTS = $(SRC_MAKEPOINTS:.cpp=.o)
SRC_TETCLIP = tetclip.cpp ply.cpp cmdhelper.cpp trimesh.cpp debugdraw.cpp draw.cpp math/math.cpp
OBJS_TETCLIP = $(SRC_TETCLIP:.cpp=.o)

.PHONY: all
all: delaunay makepoints tetclip

%.o : %.cpp
	g++ -c -o $*.o -g -Wall -Wextra -Weffc++ -pedantic -ggdb -I. -DDEBUG -DLINUX $*.cpp

delaunay : $(OBJS_DELAUNAY)
	g++ -o delaunay $(OBJS_DELAUNAY) -lm -lGL -lGLU -lglut

makepoints : $(OBJS_MAKEPOINTS)
	g++ -o makepoints $(OBJS_MAKEPOINTS) -lm 

tetclip : $(OBJS_TETCLIP)
	g++ -o tetclip $(OBJS_TETCLIP) -lm -lGL -lGLU -lglut

.PHONY: clean
clean:
	@rm -f $(OBJS_DELAUNAY) delaunay $(OBJS_MAKEPOINTS) makepoints

.PHONY: depend
depend:
	@rm -f .depend
	$(foreach srcfile,$(SRC_DELAUNAY),g++ -MM -I. $(srcfile) >> .depend;)
	$(foreach srcfile,$(SRC_MAKEPOINTS),g++ -MM -I. $(srcfile) >> .depend;)
	$(foreach srcfile,$(SRC_TETCLIP),g++ -MM -I. $(srcfile) >> .depend;)

-include .depend

