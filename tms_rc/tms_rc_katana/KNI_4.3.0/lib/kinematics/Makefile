#===========================================================================
# Kinematics Makefile
# (C) Neuronics AG
# Jonas Haller 2009
#===========================================================================
CXX			=	g++
CXXFLAGS	+=	-c -Wall -D_REENTRANT -D__NO_MATH_INLINES -pthread  -I $(BOOST_INCLUDE_DIR) -fPIC
LD			=	g++
LDFLAGS		=	-shared -fPIC
MAKE		=	make

SUBDIRS		=	AnalyticalGuess roboop

TARGET_LIB_SO 	= 	libKinematics.so
TARGET_LIB_A	=	libKinematics.a
OUTPUT_DIR		=	../linux

BOOST_INCLUDE_DIR	=	/usr/include

KINEMATICS_LIBDIRS	=	-L./roboop -L./AnalyticalGuess/lib 
UTIL_LIBS		=	-lm -lpthread
KIN_LIBFILES	=	-lroboop -lnewmat -lAnalyticalGuess

KIN_LIBS		=	roboop/libnewmat.a roboop/libroboop.a AnalyticalGuess/lib/libAnalyticalGuess.a

INCDIR			=	.
KIN_INCDIR		=	./roboop/source
MATH_INCDIR		=	./roboop/newmat
AG_INCDIR		=	./AnalyticalGuess/include

KINLIB_SOURCES 	= 	kinematics.cpp libKinematics.cpp


.SILENT:
all: build
build: subdirs $(TARGET_LIB_SO) $(TARGET_LIB_A)

%.o: %.cpp
	echo ""
	echo "=================================="
	echo "Compiling: $@ ..."
	$(CXX) $(CXXFLAGS) -I $(INCDIR) -I $(KIN_INCDIR) -I $(MATH_INCDIR) -I $(AG_INCDIR) $< 
	echo "... done"
	echo "=================================="
	echo ""

$(TARGET_LIB_SO): $(KINLIB_SOURCES:.cpp=.o)
	echo ""
	echo "=================================="
	echo "Building: $@ ..."
	$(LD) $(LDFLAGS) -o $(OUTPUT_DIR)/$@ $(KINLIB_SOURCES:.cpp=.o) $(UTIL_LIBS) $(KINEMATICS_LIBDIRS) $(KIN_LIBFILES)
	echo "... done"
	echo "=================================="
	echo ""

$(TARGET_LIB_A): $(KINLIB_SOURCES:.cpp=.o)
	echo "=================================="
	echo -n "Building: $@ ...\n"
	$(AR) $(ARFLAGS) $@ $(KINLIB_SOURCES:.cpp=.o)
	@rm -rf .ar_tmp
	@mkdir .ar_tmp; cd .ar_tmp; \
	for arfile in $(KIN_LIBS) ; do \
		$(AR) x ../$$arfile; \
		$(AR) q ../$@ *; \
		rm *; \
	done
	@cd ..
	@rm -r .ar_tmp
	ranlib $@
	mv $@ $(OUTPUT_DIR)/$@
	echo "... done"
	echo "=================================="

############################################################################
subdirs:
	@for i in $(SUBDIRS); do \
	echo "make all in $$i..."; \
	(cd $$i; $(MAKE)); done
############################################################################
clean:
	echo ""
	echo "=================================="
	echo "cleaning up ...:"
	@for i in $(SUBDIRS); do \
	echo "make clean in $$i..."; \
	(cd $$i; $(MAKE) $(MFLAGS) clean); \
	echo "...done"; done
	rm -rf *.o *~ $(OUTPUT_DIR)/$(TARGET_LIB_SO) $(OUTPUT_DIR)/$(TARGET_LIB_A) 2>/dev/null
	echo "... done cleaning up"
	echo "=================================="
	echo ""
############################################################################

