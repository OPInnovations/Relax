#ifndef helper_funcs_H
#define helper_funcs_H
#include <QtGui>
#include "opi_uce_osx.h"
#include "CommonParameters.h"

qint32 setUCETime(HANDLE *comportptr);
qint32 whatInType(QString filename);
qint32 edfhdrread(QDataStream *instrp, QString *lpidp, QString *lridp,
                  QDateTime *startDTp, qint32 *numDataRecsp, qint32 *dataRecDurp,
                  qint32 *numSignalsp, QVector<QString> *labelSignalsQVp,
                  QVector<QString> *transTypeQVp, QVector<QString> *physDimQVp,
                  QVector<qint32> *physMinQVp, QVector<qint32> *physMaxQVp,
                  QVector<qint32> *digMinQVp, QVector<qint32> *digMaxQVp,
                  QVector<QString> *prefiltQVp, QVector<qint32> *sampsPerDRQVp);

void delQVs(QVector<qint64> **tsQVpp, QVector<quint8> **skpQVpp,
            QVector<quint8> **batQVpp, QVector<qint16> **adcQVpp,
            QVector<qint16> **tmpQVpp, QVector<qint16> **axQVpp,
            QVector<qint16> **ayQVpp, QVector<qint16> **azQVpp,
            QVector<qint16> **sqQVpp, QVector<quint8> **edQVpp);
qint16 calcAct(QVector<qint16> *axQVp, QVector<qint16> *ayQVp, QVector<qint16> *azQVp,
                qint32 *newstepp, qint32 *azoldp, qint32 *axoldp, qint32 *ayoldp, qint32 *az4oldp, qint32 *ax2oldp, qint32 *ay2oldp,qint32 *ax3oldp, qint32 *ay3oldp, qint32 *ax4oldp, qint32 *ay4oldp, qint32 *az8oldp,qint32 *az12oldp, qint32 *az16oldp);
qint32 getPDNlrid(QString lrid);
void delspecQVs(qint32 pdnSlot,
                QVector<qint64> **tsQVpp, QVector<quint8> **skpQVpp,
                QVector<quint8> **batQVpp, QVector<qint16> **adcQVpp,
                QVector<qint16> **tmpQVpp, QVector<qint16> **axQVpp,
                QVector<qint16> **ayQVpp, QVector<qint16> **azQVpp,
                QVector<qint16> **sqQVpp, QVector<quint8> **edQVpp);

qint32 edfDread(QDataStream *instrp, QDateTime startDT,
                qint32 numDataRecs, qint32 dataRecDur,
                qint32 numSignals, QVector<qint32> sampsPerDRQV,
                QVector<qint64> *tsQVp, QVector<quint8> *skpQVp,
                QVector<quint8> *batQVp, QVector<qint16> *adcQVp,
                QVector<qint16> *tmpQVp, QVector<qint16> *axQVp,
                QVector<qint16> *ayQVp, QVector<qint16> *azQVp,
                QVector<qint16> *sqQVp, QVector<quint8> *edQVp,
                QVector<qint64> *annOnsetTSQVp, QVector<QString> *annTextQVp);
void edfEhdropiwrite(QDataStream *out, QString *lpidp, QString *lridp,
                     QDateTime *startDTp, qint32 numDataRecs);
qint32 edfEwrite(QDataStream *outstrp, QVector<qint16> *adcQVp,
                 QVector<qint16> *tmpQVp, QVector<qint16> *axQVp,
                 QVector<qint16> *ayQVp, QVector<qint16> *azQVp,
                 QVector<qint16> *sqQVp, qint32 startDataRecCt,
                 qint64 firstFrmTS, QVector<qint64> *tsQVp,
                 QVector<qint64> *tagTSQVp, QVector<QString> *tagTextQVp);
void procQV(QVector<qint64> *tsQVp, QVector<quint8> *skpQVp,
             QVector<quint8> *batQVp, QVector<qint16> *adcQVp,
             QVector<qint16> *tmpQVp, QVector<qint16> *axQVp,
             QVector<qint16> *ayQVp, QVector<qint16> *azQVp,
             QVector<qint16> *sqQVp, QVector<quint8> *edQVp);
QString localUTCOffset(void);
qint32 maxWLMeasure100(HANDLE *comportptr);

#endif // helper_funcs_H
