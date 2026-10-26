###############################################################################
####################### <github.com/aszdrick/magefile> ########################
############################ <aszdrick@gmail.com> #############################
###############################################################################
# Copyright (c) 2017-2018 Marleson Graf

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
############################## PROJECT VARIABLES ##############################
#export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/opt/ibm/ILOG/CPLEX_Studio2211/opl/lib/x86-64_linux

MAKEFLAGS +=-rR
### SUPPORTED SUFFIXES
HEADERSUFFIXES :=h hpp
### DIRECTORIES
SRCDIR :=src
INCDIR :=include
OBJDIR :=build
BINDIR :=bin
TSTDIR :=tests
DEPDIR :=.deps
## CPOPTIMIZER-RELATED VARIABLES
# CPLEX : l'installation locale si elle existe, sinon celle du serveur (dans le dossier personnel).
# Pour une autre installation : make CPLEXDIR=/chemin/vers/CPLEX_Studio2211
CPLEX_LOCAL  :=/opt/ibm/ILOG/CPLEX_Studio2211
CPLEX_SERVER :=$(HOME)/CPLEX_Studio2211
CPLEXDIR ?=$(if $(wildcard $(CPLEX_LOCAL)),$(CPLEX_LOCAL),$(CPLEX_SERVER))
ifeq ($(wildcard $(CPLEXDIR)/cplex/include),)
$(warning CPLEX introuvable dans $(CPLEXDIR) : make CPLEXDIR=/chemin/vers/CPLEX_Studio2211)
endif
CPOPTIMIZERDIR=$(CPLEXDIR)/cpoptimizer
### PROGRAM-RELATED VARIABLES
# Files containing the main() function
MAINFILES :=$(SRCDIR)/main_LNS.cpp $(SRCDIR)/main_nb_vehicles.cpp
# Binaries corresponding to each file with a main() function
BINARIES  :=$(BINDIR)/main_LNS $(BINDIR)/main_nb_vehicles
# Compiler & linker flags. Build mode, chosen on the command line:
#   make                  optimized, for experiments (irace, run_tests.sh, ...)
#   make MODE=debug       -O0 -g + AddressSanitizer/UBSan, to track memory bugs
#   make MODE=profile     optimized + -pg, for gprof (each run writes gmon.out)
# Switching mode rebuilds everything automatically (see BUILDFLAGS below).
MODE     ?=release
CXX      :=g++
CXXFLAGS :=-std=c++17 -Wall -Werror -DIL_STD
ifeq ($(MODE),release)
  CXXFLAGS +=-O2 -DNDEBUG
else ifeq ($(MODE),debug)
  CXXFLAGS +=-O0 -g -fsanitize=address,undefined -fno-omit-frame-pointer
else ifeq ($(MODE),profile)
  CXXFLAGS +=-O2 -g -DNDEBUG -pg
else
  $(error Unknown MODE '$(MODE)': use release, debug or profile)
endif
#LDFLAGS := -L$(CPLEXDIR)/cplex/lib/x86-64_linux/static_pic \
#           -L$(CPLEXDIR)/concert/lib/x86-64_linux/static_pic \
#           -L$(CPLEXDIR)/cpoptimizer/lib/x86-64_linux/static_pic \
#           -L$(CPLEXDIR)/opl/lib/x86-64_linux/static_pic \
#           -L$(CPLEXDIR)/opl/lib/x86-64_linux \
#           -L$(CPLEXDIR)/opl/bin/x86-64_linux/ -L/usr/lib/x86_64-linux-gnu/
#LDLIBS := -lm -licuuc -licui18n -licuio -licudata -lpthread -ldl \
#          -lilocplex -lconcert -lcp -lopl -liljs -lcplex


LDFLAGS  := -L$(CPLEXDIR)/cplex/lib/x86-64_linux/static_pic -L$(CPLEXDIR)/concert/lib/x86-64_linux/static_pic -L$(CPLEXDIR)/cpoptimizer/lib/x86-64_linux/static_pic -L$(CPLEXDIR)/opl/lib/x86-64_linux/static_pic -L$(CPLEXDIR)/opl/bin/x86-64_linux/ -L/usr/lib/x86_64-linux-gnu/
LDLIBS   :=-lilocplex -lconcert -lcplex -lm -licuuc -licui18n -licuio -licudata -lpthread -ldl # -loplnl1   -lconfig -lconfig++  -lcplex2211 -lconcert -lopl -lcp -lilocplex -liljs

# -isystem: CPLEX's own headers are not held to -Wall -Werror.
INCLUDE  :=-I$(INCDIR) -isystem $(CPLEXDIR)/cplex/include -isystem $(CPLEXDIR)/concert/include
### TESTS-RELATED VARIABLES
# Files containing the main() function
TMAINFILES :=$(wildcard $(TSTDIR)/*.cpp)
# Binaries corresponding to each file with a main() function
TBINARIES  :=$(patsubst $(TSTDIR)/%.cpp,$(BINDIR)/%,$(TMAINFILES))
# Compiler & linker flags
TCXXFLAGS :=
TLDFLAGS  :=
TLDLIBS   :=
TINCLUDE  :=
### MAKEFILE CONTROL VARIABLES
# Debug flag, if != 0 deactivates all supressed echoing
DEBUG :=1

############################# AUTOMATIC VARIABLES #############################
# Dependency files
MAINDEPS :=$(patsubst %.cpp,$(DEPDIR)/%.d,$(MAINFILES))
TMAINDEPS :=$(patsubst %.cpp,$(DEPDIR)/%.d,$(TMAINFILES))
MDEPS :=$(MAINDEPS) $(TMAINDEPS)
NDEPS :=$(filter-out $(MDEPS),$(shell find $(DEPDIR) -name *.d 2> /dev/null))
# Phony calls to build specific binaries
CALLS :=$(patsubst $(BINDIR)/%,%,$(BINARIES))
TCALLS :=$(patsubst $(BINDIR)/%,%,$(TBINARIES))
ALLCALLS :=$(CALLS) $(TCALLS)
# Autogenerated directories
AUTODIR :=$(BINDIR) $(SRCDIR) $(INCDIR)
# Auxiliar variables
sh_nop :=:
empty :=
space :=$(empty) $(empty)
# No operation of shell script
# Command to print status messages
INFO :=@echo
# Patterns to filter all header files
HEADERPATTERNS :=$(addprefix %.,$(HEADERSUFFIXES))
# Creation of variables associating each main file with a program
MAINEXECVARS :=$(addsuffix _EXEC:=,$(basename $(MAINFILES) $(TMAINFILES)))
MAINEXECVARS :=$(join $(MAINEXECVARS), $(BINARIES) $(TBINARIES))
$(foreach var,$(MAINEXECVARS),$(eval $(var)))
SOURCES =$(shell find $(SRCDIR) -name '*.cpp' 2> /dev/null)
SOURCES +=$(shell find $(TSTDIR) -name '*.cpp' 2> /dev/null)
OBJECTS :=$(eval OBJECTS:=$(patsubst %.cpp,$(OBJDIR)/%.o,$(SOURCES)))$(OBJECTS)
# Arguments: target string, output name, input name
makedep = $(CXX) $(CXXFLAGS) $(INCLUDE) -MM -MG -MT $1 -MF "$2" $3
# Arguments: file to be written, target file, objects variable name
define makedeps
	$(info [makedep] $2 -> .d)
	$(shell mkdir -p $(dir $1))
	$(shell $(call makedep,"$(OBJDIR)/$3.o $1",$1,$2))
	$(eval TARGET_OBJ:=$(patsubst %.cpp,$(OBJDIR)/%.o,$2))
	$(eval $3_EDGES:=$(abspath $(filter $(HEADERPATTERNS),$(shell cat $1))))
	$(eval $3_EDGES:=$(patsubst $(abspath $(INCDIR))%,$(SRCDIR)%,$($3_EDGES)))
	$(eval $3_EDGES:=$(patsubst %,$(OBJDIR)/%.o,$(basename $($3_EDGES))))
	$(eval HPPSONLY:=$(filter-out $(OBJECTS),$($3_EDGES)))
	$(eval $3_EDGES:=$(filter $($3_EDGES),$(OBJECTS)))
	$(eval $3_EDGES:=$(filter-out $(TARGET_OBJ),$($3_EDGES)))
	$(eval STEMLIST:=$(patsubst $(OBJDIR)/%.o,%,$($3_EDGES)))
	$(eval HPPSONLY:=$(patsubst $(OBJDIR)/%.o,%,$(HPPSONLY)))
	$(eval $3_EDGES:=$(patsubst $(OBJDIR)/%.o,%_EDGES,$($3_EDGES)))
	$(eval NDEPS+=$1)
	$(file >> $1,$1: $(patsubst %,%.cpp,$(STEMLIST) $(HPPSONLY)))
	$(file >> $1,$3_EDGES =$$(eval $3_EDGES:=$(TARGET_OBJ)\
$($3_EDGES))$$($3_EDGES))
	$(file >> $1,$3_OBJS =$$(eval RET:=)$$(eval $3_OBJS:=$$(call\
	 expand_deps,$3_EDGES)$$(RET))$$($3_OBJS))
	$(if $($3_EXEC),$(file >> $1,$($3_EXEC): $$($3_OBJS)))
	$(foreach s,$(STEMLIST),$(if $(filter-out $(NDEPS),$(DEPDIR)/$(s).d),
		$(call makedeps,$(DEPDIR)/$(s).d,$(s).cpp,$(s))))
endef
# Receives a *_EDGES list to expand
# *_EDGES list value: *.o [other *_EDGES lists]
define expand_deps
	$(foreach list,$1,\
		$(eval RG1:=$(firstword $($(list))))\
		$(if $(filter-out $(RET),$(RG1)),\
			$(eval RET+=$(RG1))\
			$(call expand_deps,$(filter-out $(RG1),$($(list))))))
endef
# If DEBUG is not active (i.e, equals 1), set supressing character (@)
ifeq ($(DEBUG),0)
SILENT :=@
endif

.PHONY: all makedir clean distclean tests $(ALLCALLS) FORCE


################################# MAIN RULES ##################################
all: makedir $(BINARIES)

$(ALLCALLS): %: $(BINDIR)/%

$(BINARIES) $(TBINARIES):
	$(INFO) "[linking] $@"
	$(SILENT) mkdir -p $(dir $@)
	$(SILENT) $(CXX) $(CXXFLAGS) $(LDFLAGS) $(INCLUDE) $^ $(LDLIBS) -o $@

$(OBJDIR)/%.o: %.cpp $(OBJDIR)/.buildflags
	$(INFO) "[  $(CXX)  ] $< -> .o"
	$(SILENT) mkdir -p $(OBJDIR)/$(*D)
	$(SILENT) $(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@

# Records the compiler flags of the last build: when they change (e.g. a new
# MODE), every object is rebuilt instead of mixing objects from two modes.
$(OBJDIR)/.buildflags: FORCE
	$(SILENT) mkdir -p $(OBJDIR)
	$(SILENT) echo '$(CXX) $(CXXFLAGS) $(INCLUDE)' | cmp -s - $@ || echo '$(CXX) $(CXXFLAGS) $(INCLUDE)' > $@

$(DEPDIR)/%.d: %.cpp
	$(SILENT) $(sh_nop) $(call makedeps,$@,$<,$*)

makedir: | $(AUTODIR)

$(AUTODIR):
	$(INFO) "[ mkdir ] Creating directory '$@'"
	$(SILENT) mkdir -p $@

################################ TESTS RULES ##################################
tests: makedir $(TBINARIES)

$(TBINARIES): LDLIBS +=$(TLDLIBS)

$(TBINARIES): LDFLAGS +=$(TLDFLAGS)

$(OBJDIR)/$(TSTDIR)/%.o: CXXFLAGS +=$(TCXXFLAGS)

$(OBJDIR)/$(TSTDIR)/%.o: INCLUDE +=$(TINCLUDE)

################################ CLEAN RULES ##################################
clean:
	$(SILENT) rm -rf $(OBJDIR)
	$(SILENT) rm -rf $(BINDIR)

distclean: clean
	$(SILENT) rm -rf $(DEPDIR)

################################ PREREQUISITES ################################
# Do not include list of dependencies with clean rules
ifeq ($(filter-out all $(CALLS) $(BINARIES),$(MAKECMDGOALS)),)
  -include $(NDEPS)
  -include $(MAINDEPS)
else
  ifeq ($(filter-out tests $(TCALLS) $(TBINARIES),$(MAKECMDGOALS)),)
    -include $(NDEPS)
    -include $(TMAINDEPS)
  endif
endif


