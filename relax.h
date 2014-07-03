#ifndef RELAXLIVE_H
#define RELAXLIVE_H


/***
  *
  * Includes
  *
  */
#include <QtGui>
#include "opi_uce_win.h"
#include "faderwidget.h"
#include "qqwidget.h"
#include "FFT/qfouriertransformer.h"
#include "FFT/qcomplexnumber.h"
#include "helper_funcs.h"
#include "postureviewer/twodaccelviewer.h"


/***
  *
  * Definitions
  *
  */



namespace Ui {
class ReLax;
}

class ReLax : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit ReLax(QWidget *parent = 0);
    ~ReLax();

    
protected:
    void timerEvent(QTimerEvent *event);   // reimplemented function
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *r);
    void closeEvent(QCloseEvent *);
    void dragEnterEvent(QDragEnterEvent *e);
private slots:
    void on_modeSli_valueChanged(int value);
    void fadeInWidget(int index);
    void on_startPB_clicked(bool checked);
    void on_savedBrowsePB_clicked();
    void on_signalTimeSlider_valueChanged(int value);
    void on_liveLevelSli_valueChanged(int value);
    void on_signalGainUpPB_clicked();
    void on_signalGainDnPB_clicked();
    void on_spectEnablePB_clicked(bool checked);
    void on_GraphicsDrawCheckBox_clicked();
    void on_signalTimePositionSlider_valueChanged(int position);

private:
    Ui::ReLax *ui;
    HANDLE comPort;

    // Signal
    QQueue<qint16> signalDataToDrawQQ;  // live mode data
    qint32 signalDrawCount;             // current place to draw new signal data
    qint16 signalbegin;    //decide the location where starts to draw
    float signalincrease;  //decide the gap between every points(x-axis)
    float scalerate; //decide the scale of the signal(y-axis),scalerate=(scene'height)/(signal range you want to show)
    qint16 screenmiddle;   //calculate where to put the x-axis(the middle of the scene)
    qint16 signalmiddle; //design what is the middle value of your input siganl
    qint32 screenShowTime; // be initialized as INITIALADC
    bool firstTimeDrawsignal;
    QVector<qint16> signalDataQvp;
    QGraphicsScene *signalQGSp;
    int nowvalue,beforevalue;
    float signalZoomRatio;
    int signalstartindex;

    // Spectrogram
    QQueue<qint16> signalDataToFFTQQ; // queue for holding signalData that will need to be FFT'd for spectrogram
    QVector<float> spectDataToDrawQV;  // live mode data, type right?
    double effected_by_ko_co_fft[NUMPATHFFT/2];  // to save the result fft data effected by ko co for drawFFT Function
    qint32 spectDrawCount;              // current place to draw new spect data
    QGraphicsScene *spectQGSp;
    int button; //decide the x-axis of fft screen
    qint16 spectbegin;    //decide the location where starts to draw
    qint32 numScalefft;
    QColor calcSpectRectColor(double value);
    QFourierTransformer myQFT;

    // LED
    QVector<quint16> uceFFTDataToProcessQV; // vector for holding uce data for deciding Relax state
    QQueue<quint8> ledDataToDrawQQ;    // live mode data
    qint32 ledDrawCount;                // current place to draw new led data
    quint16 lCalc, bCalc, gCalc, mCalc, gmCalc, bmCalc; // persistent variables for relax state algorithm
    quint8 stRelax_now, stRelax_prev;
    quint16 rlevel, thxx, thmm, thgm, thbm, offll, offm,; // for state algorithm calc
    quint16 thx, thm, offl, th3gm, th2gm, th1gm, th3bm, th2bm, th1bm; // for Controller state algorithm
    float rls1, rls2, rls3, rlsm; //for RLEVEL scaling
    QGraphicsScene *ledQGSp;
    qint32 stRelax_pktCt, stRelax_accum;

    // For writing to EDF
    QFile *outQFp;
    QDataStream *outQDSp;
    QString localPatientID, localRecordID;
    qint32 wroteDRCt;
    QDateTime stQDT;
    QVector<qint64> tsQV;
    QVector<quint8> skpQV;
    QVector<quint8> batQV;
    QVector<qint16> adcQV;
    QVector<qint16> tmpQV;
    QVector<qint16> axQV;
    QVector<qint16> ayQV;
    QVector<qint16> azQV;
    QVector<qint16> sqQV;
    QVector<quint8> edQV;
    qint64 prevFrmTS, firstFrmTS;

    // Posture viewer
    twoDaccelviewer *TDVp;
    QVector<qint16> accxQV,accyQV,acczQV;

    // LIVEMODE stuff
    QBasicTimer timer;
    bool configDoneFlag;
    qint64 timerEventCt; // for determining if certain actions should happen in timerEvent
    qint32 noPktTimerCt;    // for keeping track of how many times timer called without receiving in a packet when actively getting data

    // SAVEDMODE stuff
    qqwidget *qqscenep; //saved draw fft qqwidget
    quint8 tsepdn;
    qint32 newstep, azold, axold, ayold, az4old, ax2old, ay2old, ax3old, ay3old, ax4old, ay4old, az8old, az12old, az16old; // for activity
    void drawSavedSpect();
    void drawSavedSignal();
    bool convertedf();
    long index_begin;
    float show_num_of_seconds ;
    //end
    //mouse move
    int pressx,pressy,releasex,releasey;  //initial 0
    int getmouseflag;//initial-1; //A=1 F=6
    int changablemiddle;

    // RLSCORE stuff
    void getRelaxData();
    void resetRelaxData();
};

#endif // RELAXLIVE_H
