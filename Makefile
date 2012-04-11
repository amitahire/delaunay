ifndef config
	config=debug
endif

COMPILE=g++
NAME_DELAUNAY=delaunay
NAME_MAKEPOINTS=makepoints
CPPFLAGS=-MMD -MP -Wall -Wextra -Weffc++ -pedantic -fno-rtti -fno-exceptions -fno-math-errno -ffast-math -I. -DDEBUG -DLINUX
DEFINES=

ifeq ($(config),release)
	DEFINES += -DRELEASE
	CPPFLAGS += -O3 $(DEFINES)
	TARGETDIR = .
	TARGET_DELAUNAY = $(NAME_DELAUNAY)-z
	TARGET_MAKEPOINTS = $(NAME_MAKEPOINTS)-z
	OBJDIR = obj/release
endif

ifeq ($(config),debug)
	DEFINES += -DDEBUG
	CPPFLAGS += -g -ggdb $(DEFINES)
	TARGETDIR = .
	TARGET_DELAUNAY = $(NAME_DELAUNAY)-d
	TARGET_MAKEPOINTS = $(NAME_MAKEPOINTS)-d
	OBJDIR = obj/debug
endif

LIBS = -lm -lGL -lGLU -lglut

OBJ_DELAUNAY := \
	$(OBJDIR)/delaunay.o \
	$(OBJDIR)/debugdraw.o \
	$(OBJDIR)/math/math.o \
	$(OBJDIR)/sparsegrid.o \
	$(OBJDIR)/triangulator.o \
	$(OBJDIR)/draw.o \
	$(OBJDIR)/cmdhelper.o \

OBJ_MAKEPOINTS := \
	$(OBJDIR)/makepoints.o \
	$(OBJDIR)/cmdhelper.o \

OBJS_DELAUNAY = $(SRC_DELAUNAY:.cpp=.o)
OBJS_MAKEPOINTS = $(SRC_MAKEPOINTS:.cpp=.o)

.PHONY: clean

all: $(TARGETDIR) $(OBJDIR) $(OBJDIR)/math $(TARGET_DELAUNAY) $(TARGET_MAKEPOINTS)
	@:

$(TARGET_DELAUNAY) : $(OBJ_DELAUNAY)
	$(COMPILE) -o  $(TARGET_DELAUNAY) $(OBJ_DELAUNAY) $(LIBS)

$(TARGET_MAKEPOINTS) : $(OBJ_MAKEPOINTS)
	$(COMPILE) -o  $(TARGET_MAKEPOINTS) $(OBJ_MAKEPOINTS) $(LIBS)

$(TARGETDIR):
	mkdir -p $(TARGETDIR)

$(OBJDIR):
	mkdir -p $(OBJDIR)

$(OBJDIR)/math:
	mkdir -p $(OBJDIR)/math

clean:
	rm -f $(TARGET_DELAUNAY) $(TARGET_MAKEPOINTS)
	rm -rf $(OBJDIR)

$(OBJDIR)/%.o : %.cpp
	$(COMPILE) -c -o $@ $(CPPFLAGS) $< 

$(OBJDIR)/math/%.o : math/%.cpp
	$(COMPILE) -c -o $@ $(CPPFLAGS) $< 

.PHONY: depend
depend:
	@rm -f .depend
	$(foreach srcfile,$(SRC_DELAUNAY),g++ -MM -I. $(srcfile) >> .depend;)
	$(foreach srcfile,$(SRC_MAKEPOINTS),g++ -MM -I. $(srcfile) >> .depend;)

-include $(OBJ_DELAUNAY:%.o=%.d)
-include $(OBJ_MAKEPOINTS:%.o=%.d)

