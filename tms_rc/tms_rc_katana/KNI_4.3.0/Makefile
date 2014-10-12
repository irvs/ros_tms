#===========================================================================
# KNI root Makefile
# copyright Neuronics Switzerland 2005-2009
# Authors: see AUTHORS file
#===========================================================================

TARGETS			=	lib/kinematics src/common src/Base src/InvKin src/LM src/kni_wrapper py demo
TARGETS_B		=	$(foreach target,$(TARGETS),$(target).build)
TARGETS_C		=	$(foreach target,$(TARGETS),$(target).clean)
NEURONICS_DOC_DIR	=	doc/neuronics-doc/source

.SILENT:

.PHONY: all doc

all:
	$(MAKE) $(TARGETS_B)

clean:
	$(MAKE) $(TARGETS_C)

$(TARGETS_B):
	echo ""
	echo "==========================================================="
	echo "---  $(@)"
	echo "-----------------------------------------------------------"
	$(MAKE) -C $(@:.build=)
	echo "==========================================================="

$(TARGETS_C):
	echo ""
	echo "==========================================================="
	echo "---  $(@)"
	echo "-----------------------------------------------------------"
	$(MAKE)	-C $(@:.clean=) clean

doc.clean:
	echo ""
	echo "==========================================================="
	echo "---  $(@)"
	echo "-----------------------------------------------------------"
	rm -Rf ./doc/html
	rm -Rf ./doc/rtf
	rm -Rf ./doc/latex
	$(MAKE) -C $(NEURONICS_DOC_DIR) clean

doc:	
	echo ""
	echo "==========================================================="
	echo "---  $(@)"
	echo "-----------------------------------------------------------"
	doxygen	Doxyfile

doc-neuronics: doc 
	ln -sf $(PWD)/doc/latex/*.tex  $(PWD)/$(NEURONICS_DOC_DIR)  
	ln -sf $(PWD)/doc/latex/*.pdf  $(PWD)/$(NEURONICS_DOC_DIR)
	ln -sf $(PWD)/doc/latex/*.eps  $(PWD)/$(NEURONICS_DOC_DIR)
	cd $(NEURONICS_DOC_DIR); ./trunkfile.py
	$(MAKE) -C $(NEURONICS_DOC_DIR) install

doc-neuronics-deploy: doc-neuronics
	$(MAKE) -C $(NEURONICS_DOC_DIR) deploy
