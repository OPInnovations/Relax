#ifndef GLWIDGET_H
#define GLWIDGET_H
#include <QDebug>
#include "CommonParameters.h"
#include "QWidget"
#include "QGraphicsScene"
#include "QPen"
#include "QBrush"

QT_BEGIN_NAMESPACE
class QPaintEvent;
class QWidget;
QT_END_NAMESPACE
class qqwidget : public QWidget
{
    int numScalefft;
    double fft_total;
    int counteffectedcolum;
    bool firstdraw; //default true
public:
    qqwidget(QWidget *parent,qint8 window_width,qint8 window_height);
    void setData(int numScalefftp,int colum);
    bool todraw;
    int countdraw;
    int shifteindex ;
    double effected_by_ko_co_fft[512][NUMPATHFFT/2];  // to save the result fft data effected by ko co for drawFFT Function
    float increase_log[NUMPATHFFT/2+1]; //decide the gap between every points(x-axis)in LOG scale & as FFT_equalization
protected:
    void paintEvent(QPaintEvent *event);
    void closeEvent(QCloseEvent *);
};

#endif // GLWIDGET_H
