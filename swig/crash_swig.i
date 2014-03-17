/* -*- c++ -*- */

#define CRASH_API

%include "gnuradio.i"			// the common stuff

//load generated python docstrings
%include "crash_swig_doc.i"

%{
#include "crash/crash_usrp_source.h"
#include "crash/crash_usrp_sink.h"
%}


%include "crash/crash_usrp_source.h"
GR_SWIG_BLOCK_MAGIC2(crash, crash_usrp_source);
%include "crash/crash_usrp_sink.h"
GR_SWIG_BLOCK_MAGIC2(crash, crash_usrp_sink);
