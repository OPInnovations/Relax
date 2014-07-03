#-------------------------------------------------
#
# Project created by QtCreator 2013-06-27T16:23:26
#
#-------------------------------------------------

QT       += core gui

TARGET = ReLax
TEMPLATE = app


SOURCES += main.cpp\
        relax.cpp \
    opi_uce_win.cpp \
    faderwidget.cpp \
    qqwidget.cpp \
    FFT/qwindowfunction.cpp \
    FFT/qfouriervariablecalculator.cpp \
    FFT/qfouriertransformer.cpp \
    FFT/qfourierfixedcalculator.cpp \
    FFT/qfouriercalculator.cpp \
    FFT/qcomplexnumber.cpp \
    postureviewer/twodaccelviewer.cpp \
    helper_funcs.cpp

HEADERS  += relax.h \
    opi_uce_win.h \
    faderwidget.h \
    CommonParameters.h \
    qqwidget.h \
    FFT/qwindowfunction.h \
    FFT/qfouriervariablecalculator.h \
    FFT/qfouriertransformer.h \
    FFT/qfourierfixedcalculator.h \
    FFT/qfouriercalculator.h \
    FFT/qcomplexnumber.h \
    FFT/OscSinCos.hpp \
    FFT/OscSinCos.h \
    FFT/FFTRealUseTrigo.hpp \
    FFT/FFTRealUseTrigo.h \
    FFT/FFTRealSelect.hpp \
    FFT/FFTRealSelect.h \
    FFT/FFTRealPassInverse.hpp \
    FFT/FFTRealPassInverse.h \
    FFT/FFTRealPassDirect.hpp \
    FFT/FFTRealPassDirect.h \
    FFT/FFTRealFixLenParam.h \
    FFT/FFTRealFixLen.hpp \
    FFT/FFTRealFixLen.h \
    FFT/FFTReal.hpp \
    FFT/FFTReal.h \
    FFT/DynArray.hpp \
    FFT/DynArray.h \
    FFT/def.h \
    FFT/Array.hpp \
    FFT/Array.h \
    postureviewer/twodaccelviewer.h \
    helper_funcs.h

FORMS    += relax.ui \
    postureviewer/twodaccelviewer.ui

RESOURCES += \
    ReLax.qrc


RC_FILE += images/myIcon.rc

OTHER_FILES +=

