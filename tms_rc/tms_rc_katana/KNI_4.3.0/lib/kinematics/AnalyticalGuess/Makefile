#===========================================================================
# AnalyticalGuess
# Root Makefile
# copyright: Neuronics AG, Switzerland, 2008
# Author: Jonas Haller 05/2008
#===========================================================================


TARGETS			=	src
TARGETS_B		=	$(foreach target,$(TARGETS),$(target).build)
TARGETS_C		=	$(foreach target,$(TARGETS),$(target).clean)

.SILENT:

.PHONY: all doc

all:
	$(MAKE) $(TARGETS_B)

clean:
	$(MAKE) $(TARGETS_C)

cross: 
	$(MAKE) CXX=/opt/eldk/usr/bin/ppc_6xx-g++ LD=/opt/eldk/usr/bin/ppc_6xx-g++

$(TARGETS_B):
	echo ""
	echo "___________________________________________________________"
	echo "$(@)"
	echo "___________________________________________________________"
	$(MAKE) -C $(@:.build=)
	echo ""

$(TARGETS_C):
	echo ""
	echo "___________________________________________________________"
	echo "$(@)"
	echo "___________________________________________________________"
	$(MAKE)	-C $(@:.clean=) clean

doc.clean:
	echo ""
	echo "___________________________________________________________"
	echo "$(@)"
	echo "___________________________________________________________"
	rm -Rf ./doc/html

doc:
	echo ""
	echo "___________________________________________________________"
	echo "$(@)"
	echo "___________________________________________________________"
	doxygen	Doxyfile

