# -----------------------------------------------------------------------------
# Makefile for the generation of a pdf dokument files from latex files
#
# Usage: place your dot files with this Makefile into a directory
#        and execute "make".
#
# by Lukas Burger, Thomas Popp, 2003 - 2007
# -----------------------------------------------------------------------------
TARGET = manual

# spelling options
ASPELLLANG   := --lang=de
ASPELLPWS    := -p aspell-WordList.pws
ASPELLPREPL  := --repl aspell--ReplaceList.prepl
ASPELLOPT    := --add-tex-command="hypersetup p,lstset p,fancyhead oo,bibliographystyle p,PassOptionsToPackage pp,pdfinfo p,todo p"

# installing options (getting document name from latex files)
INSTALL	:= ..
DOCNUM  := $(shell cat $(TARGET).tex | grep \\myDocnum | sed 's/.*{\(.*\)}.*/\1/')
VERSION := $(shell cat $(TARGET).tex | grep \\myVersion | sed 's/.*{\(.*\)}.*/\1/')
TITLE	:= $(shell cat $(TARGET).tex | grep \\myTitle | sed 's/.*{\(.*\)}.*/\1/' | sed 's/ /_/g' | sed 's/\,/_/g')
SUBTITLE := $(shell cat $(TARGET).tex | grep \\mySubTitle | sed 's/.*{\(.*\)}.*/\1/' | sed 's/ /_/g' | sed 's/\,/_/g')
PRODUCT  := $(shell cat $(TARGET).tex | grep \\myProduct | sed 's/.*{\(.*\)}.*/\1/' | sed 's/ /_/g' | sed 's/\,/_/g')

ifeq ($(DOCNUM),)
  SEP =
else
  SEP = -
endif

DOCNAME := $(INSTALL)/$(DOCNUM)$(SEP)$(PRODUCT)-$(TITLE)-$(SUBTITLE)-v$(VERSION).pdf

TEXFILES := $(wildcard *.tex)

AUTOIMAGEDIRS := tgif dia octave dot

ifeq ($(V),1)
  Q     =
  QECHO = @\#
else
  Q     = @
  QECHO = @echo
endif

all: $(TARGET).pdf

new:
	rm -f $(TARGET).pdf

install: all
	cp $(TARGET).pdf $(DOCNAME)

deploy: install
	$(MAKE) distclean
	svn add $(DOCNAME)
	svn commit . $(DOCNAME) -m "adding document $(DOCNUM) $(TITLE) v$(VERSION)"

auto-images:
	$(Q) for i in $(AUTOIMAGEDIRS); do \
	       if [ -d $$i ]; then \
	         $(MAKE) -C $$i; \
	       fi; \
	     done

# PDF
${TARGET}.pdf: $(TEXFILES) auto-images
	echo "Running latex..."
	pdflatex ${TARGET}.tex
	if [ -f bibliography.tex ]; then \
	    echo "Running bibtex..."; \
	    bibtex ${TARGET}; \
	fi
	echo "Rerunning latex...."
	pdflatex ${TARGET}.tex
	latex_count=5 ; \
	while egrep -s 'Rerun (LaTeX|to get cross-references right)' ${TARGET}.log && [ $$latex_count -gt 0 ] ;\
	    do \
	      echo "Rerunning latex...." ;\
	      pdflatex ${TARGET}.tex ;\
	      latex_count=`expr $$latex_count - 1` ;\
	    done

# spell checker
spell:  $(subst .tex,.tex.bak,$(wildcard *.tex))

# spell checker rule
%.tex.bak: %.tex
	aspell --mode=tex $(ASPELLLANG) $(ASPELLPWS) $(ASPELLPREPL) $(ASPELLOPT) check $<

# garbage collectors
.PHONY:	clean cleanimages distclean mrproper
clean:
	$(QECHO) -n "Cleaning up. "
	$(Q) rm -f *~ *.tex.bak *.tex.backup *.aux *.log *.out *.blg *.bbl *.brf *.toc *.lof *.lot *.dvi report.pdf
	$(QECHO) "[ok]"
	$(Q) rm `find . -type l`
	$(Q) rm filelist.tex

cleanimages:
	$(Q) for i in $(AUTOIMAGEDIRS); do \
	       if [ -d $$i ]; then \
	         $(MAKE) -C $$i clean; \
	       fi \
	     done

distclean:	clean cleanimages

mrproper: distclean
	rm -f $(TARGET).pdf
