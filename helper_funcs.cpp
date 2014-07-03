#include "helper_funcs.h"


/***
  *	Set the Controller time to current time.
  *  Assumes the comport has already been opened by SDK.
  *	Inputs:
  *		comportptr, pointer to handle
  *	Returns:
  *      0, if successful
  *      -1, if error
  */
qint32 setUCETime(HANDLE *comportptr)
{
    qint64 ucRefEpochMSecs, ucSetTS;
    QDateTime currDT, refDT;
    int timeStamp[6];

    // Conversion to QDateTime, ref date & time for all sensors is 2012/sep/28 08:00:00.000
    currDT = QDateTime::currentDateTime();
    refDT = QDateTime::fromString("20120928080000000","yyyyMMddhhmmsszzz");
    ucRefEpochMSecs = currDT.toMSecsSinceEpoch() - refDT.toMSecsSinceEpoch();
    ucSetTS = ucRefEpochMSecs*UCERTCFREQ/1000;

    // Set Timestamp, need to correct and put correct date time in
    timeStamp[0] = (ucSetTS >> 40) & 0xFF;
    timeStamp[1] = (ucSetTS >> 32) & 0xFF;
    timeStamp[2] = (ucSetTS >> 24) & 0xFF;
    timeStamp[3] = (ucSetTS >> 16) & 0xFF;
    timeStamp[4] = (ucSetTS >> 8) & 0xFF;
    timeStamp[5] = (ucSetTS >> 0) & 0xFF;

    if(opiuce_setpktts(comportptr, timeStamp)) return -1;
    else return 0;
}


/***
  * Read in an edf file that is generic opi data. Assumes data is in right format.
  * Inputs:
  *     all data will be appended to end of qvectors
  * Returns:
  *     non-negative number indicating number of data records read
  *     -1, error
  */
qint32 edfDread(QDataStream *instrp, QDateTime startDT,
                               qint32 numDataRecs, qint32 dataRecDur,
                               qint32 numSignals, QVector<qint32> sampsPerDRQV,
                               QVector<qint64> *tsQVp, QVector<quint8> *skpQVp,
                               QVector<quint8> *batQVp, QVector<qint16> *adcQVp,
                               QVector<qint16> *tmpQVp, QVector<qint16> *axQVp,
                               QVector<qint16> *ayQVp, QVector<qint16> *azQVp,
                               QVector<qint16> *sqQVp, QVector<quint8> *edQVp,
                               QVector<qint64> *annOnsetTSQVp,
                               QVector<QString> *annTextQVp)
{
    qint16 tempadcs[ADCLEN*FRMSPERSEC*EDFDRDURSEC];
    qint16 tempaxs[ACCLEN/4*FRMSPERSEC*EDFDRDURSEC];
    qint16 tempays[ACCLEN/4*FRMSPERSEC*EDFDRDURSEC];
    qint16 tempazs[ACCLEN*FRMSPERSEC*EDFDRDURSEC];
    qint16 temptmps[TMPLEN*FRMSPERSEC*EDFDRDURSEC];
    qint16 tempsqs[1*FRMSPERSEC*EDFDRDURSEC];
    quint8 tempanns[128];
    qint32 i, j, dataRecCt;
    qint64 prevTS, tempTS;
    QStringList annsQL, annQL;
    qint32 newstep, azold, axold, ayold, az4old, ax2old, ay2old, ax3old, ay3old, ax4old, ay4old, az8old, az12old, az16old;
    newstep = 1;//init
    azold = 0;
    axold = 0;
    ayold = 0;
    az4old = 0;
    ax2old = 0;
    ay2old = 0;
    ax3old = 0;
    ay3old = 0;
    ax4old = 0;
    ay4old = 0;
    az8old = 0;
    az12old = 0;
    az16old = 0;
    // check to make sure things are right
    if((sampsPerDRQV.at(0) != ADCLEN*FRMSPERSEC*EDFDRDURSEC) ||
            (sampsPerDRQV.at(1) != ACCLEN/4*FRMSPERSEC*EDFDRDURSEC) ||
            (sampsPerDRQV.at(2) != ACCLEN/4*FRMSPERSEC*EDFDRDURSEC) ||
            (sampsPerDRQV.at(3) != ACCLEN*FRMSPERSEC*EDFDRDURSEC) ||
            (sampsPerDRQV.at(4) != TMPLEN*FRMSPERSEC*EDFDRDURSEC) ||
            (sampsPerDRQV.at(5) != ACCLEN/4*FRMSPERSEC*EDFDRDURSEC) ||
            ((sampsPerDRQV.at(6) != 64) && (sampsPerDRQV.at(6) != 30)))
        return -1;

    // Initialization
    prevTS = (startDT.toMSecsSinceEpoch()-QDateTime::fromString("20120928080000000","yyyyMMddhhmmsszzz").toMSecsSinceEpoch())*UCERTCFREQ/1000;
    prevTS -= ADCLEN*UCERTCFREQ/TSERTCFREQ;   // must make it previous TS
    if(prevTS < 0) prevTS = 0;
    dataRecCt = 0;

    // Read in data until end
    while(!instrp->atEnd()) // read until no more data
    {
        // adc data 8192 = sampsPerDRQV.at(0)*2bytes
        if(instrp->readRawData((char *)tempadcs, sampsPerDRQV.at(0)*2) < 0) break;  // not enough data
        if(instrp->readRawData((char *)tempaxs, sampsPerDRQV.at(1)*2) < 0) break;
        if(instrp->readRawData((char *)tempays, sampsPerDRQV.at(2)*2) < 0) break;
        if(instrp->readRawData((char *)tempazs, sampsPerDRQV.at(3)*2) < 0) break;
        if(instrp->readRawData((char *)temptmps, sampsPerDRQV.at(4)*2) < 0) break;
        if(instrp->readRawData((char *)tempsqs, sampsPerDRQV.at(5)*2) < 0) break;
        if(instrp->readRawData((char *)tempanns, sampsPerDRQV.at(6)*2) < 0) break;

        // put into qvectors because data record is complete
        for(i = 0; i < 1*FRMSPERSEC*EDFDRDURSEC; i++)
        {
            tsQVp->append(prevTS+ADCLEN*UCERTCFREQ/TSERTCFREQ);
            prevTS += ADCLEN*UCERTCFREQ/TSERTCFREQ;
            skpQVp->append(0);
            batQVp->append(1);
            for(j = 0; j < ADCLEN; j++)
                adcQVp->append(tempadcs[i*ADCLEN+j]);
            for(j = 0; j < ACCLEN/4; j++)
            {
                axQVp->append(tempaxs[i*ACCLEN/4+j]);
                ayQVp->append(tempays[i*ACCLEN/4+j]);
            }
            for(j = 0; j < ACCLEN; j++)
                azQVp->append(tempazs[i*ACCLEN+j]);
            for(j = 0; j < TMPLEN; j++)
                tmpQVp->append(temptmps[i*TMPLEN+j]);
            sqQVp->append(calcAct(axQVp, ayQVp, azQVp, &newstep, &azold, &axold, &ayold, &az4old, &ax2old, &ay2old, &ax3old, &ay3old, &ax4old, &ay4old, &az8old, &az12old, &az16old));
            edQVp->append(0);
        }
        // take care of annotations
        annsQL = QString::fromAscii((const char *)tempanns,128).split(QChar(0),QString::SkipEmptyParts);
        for(i = 0; i < annsQL.size(); i++)
        {
            annQL = annsQL.at(i).split(QChar(20),QString::SkipEmptyParts); // split each entry
            if(annQL.size() < 2) continue; // no tag entries
            // first parts is always the time
            tempTS = (qint64) (annQL.at(0).toFloat()*UCERTCFREQ+(startDT.toMSecsSinceEpoch()-QDateTime::fromString("20120928080000000","yyyyMMddhhmmsszzz").toMSecsSinceEpoch())*UCERTCFREQ/1000);
            for(j = 1; j < annQL.size(); j++)
            {
                annOnsetTSQVp->append(tempTS);
                annTextQVp->append(annQL.at(j));
            }
        }
        dataRecCt++;
    }

    return dataRecCt;
}

/***
  * Calculate activity and return a quint8 (compat. with prev. def.)
  */
qint16 calcAct(QVector<qint16> *axQVp, QVector<qint16> *ayQVp, QVector<qint16> *azQVp,
               qint32 *newstepp, qint32 *azoldp, qint32 *axoldp, qint32 *ayoldp, qint32 *az4oldp, qint32 *ax2oldp, qint32 *ay2oldp,qint32 *ax3oldp, qint32 *ay3oldp, qint32 *ax4oldp, qint32 *ay4oldp, qint32 *az8oldp,qint32 *az12oldp, qint32 *az16oldp)
{
    qint64 i,j, fastdx, fastdy, fastdz, slowdx, slowdy, slowdz, sfastdz;
    qint64 fastAct, slowAct, sfastAct, activ, z4sum, x4mean, y4mean, z16mean, xsum, ysum;
    qint16 retval;
    slowdx = 0; //init
    slowdy = 0; //init
    slowdz = 0; //init
    fastdz = 0; //init
    sfastAct = 0;
    z4sum = 0; //init
    x4mean = 0;
    y4mean = 0;
    z16mean = 0;
    xsum = 0;
    ysum = 0;
    j = (azQVp->size()); //max index
    sfastdz = (azQVp->at(j-4)- *azoldp); //current packet
    sfastAct += sfastdz*sfastdz;
    sfastdz = (azQVp->at(j-3)-azQVp->at(j-4)); //current packet
    sfastAct += sfastdz*sfastdz;
    sfastdz = (azQVp->at(j-2)-azQVp->at(j-3)); //current packet
    sfastAct += sfastdz*sfastdz;
    sfastdz = (azQVp->at(j-1)-azQVp->at(j-2)); //current packet
    sfastAct += sfastdz*sfastdz;
    xsum = axQVp->at(j/4-1); //current packet
    ysum = ayQVp->at(j/4-1); //current packet
    fastdx = xsum - *axoldp; //current vs. last packet
    fastdy = ysum - *ayoldp; //current vs. last packet
    z4sum += (azQVp->at(j-1));
    z4sum += (azQVp->at(j-2));
    z4sum += (azQVp->at(j-3));
    z4sum += (azQVp->at(j-4));
    fastdz = (z4sum - *az4oldp)/4; //average of 4
    fastAct = fastdx*fastdx + fastdy*fastdy + fastdz*fastdz; //sum of square
    slowdz = (z4sum - *az16oldp)/16; //average of 4, 4 packets apart
    x4mean = (*axoldp + *ax2oldp + *ax3oldp + *ax4oldp)/4; //mean of 4 packets
    y4mean = (*ayoldp + *ay2oldp + *ay3oldp + *ay4oldp)/4; //mean of 4 packets
    z16mean = (*az4oldp+ *az8oldp+ *az12oldp+ *az16oldp)/16; //mean of 4 packets
    slowdx = (xsum - *ax4oldp)/4; //4 packets apart
    slowdy = (ysum - *ay4oldp)/4; //4 packets apart
    slowAct = slowdx*slowdx + slowdy*slowdy + slowdz*slowdz; //sum of square

    if(slowAct<=257000) slowAct=0; //noise reduction
    if(fastAct<=257000) fastAct=0; //noise reduction
    if(sfastAct<=257000) sfastAct=0; //noise reduction
    activ = (ACTOFFSET + (ACTGAIN*(fastAct/FASTACTWEIGHT + slowAct/SLOWACTWEIGHT + sfastAct/SFASTACTWEIGHT))); // activ must be qint16
    if(activ<=1000000) activ=1000000; //set noise floor 256*256*4=256K
    activ = 6553.6*(log10(activ)-6.0);  //log10 50dB dynamic range
    if(activ > 32767) activ = 32767;    // clipping, since return value is qint16
    if(activ < 0) activ = 0; // set floor
    if(j<16 && activ>6553.6) activ=6553.6; //block initial spike >10db
    //check step using zero-crossing: newstepp=-1(neg domain), +1(pos domain), +2(pos transition detected=>add 1 step)
    for(i=4; i>0; i--)
    {
        if((azQVp->at(j-i)-xsum+ysum - z16mean+x4mean-y4mean)> 3000) //positive with hysteresis
        {
            if((*newstepp)==-1) *newstepp=2; //advance 1 step
            else if(*newstepp==-2) *newstepp=3; //advance 2 steps
        }
        else if((azQVp->at(j-i)-xsum+ysum - z16mean+x4mean-y4mean) < -3000) //negative with hysteresis
        {
            if((*newstepp)==1) *newstepp=-1; //negative
            else if(*newstepp==2) *newstepp=-2; //negative
        }
    }
    *azoldp = azQVp->at(j-1); //new value
    *az16oldp = *az12oldp; //new value
    *az12oldp = *az8oldp; //new value
    *az8oldp = *az4oldp; //new value
    *az4oldp = z4sum; //new value
    *ax4oldp = *ax3oldp; //new value
    *ax3oldp = *ax2oldp; //new value
    *ax2oldp = *axoldp; //new value
    *axoldp = xsum; //new value
    *ay4oldp = *ay3oldp; //new value
    *ay3oldp = *ay2oldp; //new value
    *ay2oldp = *ayoldp; //new value
    *ayoldp = ysum; //new value

    retval = (qint16) activ;
    return retval; //no motion <5db; slow(low) 5~10db; walk(mid) 10~20db; run(hi) 20~28db; shake(intense) >28db;
}



// Gets the PDN from the local recording ID. Usually stored in device
// field, but use regular expression so will find anywhere. If can't find
// it then will set it 255.
qint32 getPDNlrid(QString lrid)
{
    QRegExp pdnQRE;
    QRegExp pdnQRE2;
    QStringList tempQSL;
    qint32 retPDN;

    // get PDN from Device field
    pdnQRE2.setPattern("OPITS[A-Fa-f0-9]{2}");  // match OPITSxx where xx is hexadecimal digits
    pdnQRE.setPattern("OPITS[0-9]{3}");     // match OPITSxxx where xxx is decimal digits
    tempQSL = lrid.trimmed().split(QRegExp("\\s+"));
    if(tempQSL.size() > 0)
    {
        if(pdnQRE.indexIn(tempQSL.last()) > -1)
        {
            retPDN = pdnQRE.cap(0).replace("OPITS","").toInt(0, 10);
        }
        else if(pdnQRE2.indexIn(tempQSL.last()) > -1)
        {
            retPDN = pdnQRE2.cap(0).replace("OPITS","").toInt(0, 16);
        }
        else
        {
            retPDN = 255;   // unknown PDN
        }
    }
    else
    {
        retPDN = 255;   // empty so unknown PDN
    }

    return retPDN;
}
/***
  * Figure out what kind of file type
  * Inputs:
  *     filename, name of file
  * Returns:
  *     -1, if error
  *     0, if unrecognized type
  *     1, if generic tse data .edf
  */
qint32 whatInType(QString filename)
{
    QFile *infilep;
    QDataStream *instrp;
    quint8 opiHdr[OPIHDRLEN];
    qint32 i;
    QString lpid, lrid;
    QDateTime startDT;
    qint32 numDataRecs, dataRecDur;
    qint32 numSignals;
    QVector<QString> labelSignalsQV, transTypeQV, physDimQV, prefiltQV;
    QVector<qint32> physMinQV, physMaxQV, digMinQV, digMaxQV;
    QVector<qint32> sampsPerDRQV;

    // Opening of inputs/outputs and Error Checking
    infilep = new QFile(filename.trimmed());
    if (!infilep->open(QIODevice::ReadOnly))
    {
        delete infilep;
        return -1;
    }

    // open input file stream
    instrp = new QDataStream(infilep);
    instrp->setByteOrder(QDataStream::LittleEndian);

    // check if it is opi
    for(i = 0; i < OPIHDRLEN; i++)
    {
        if(instrp->atEnd()) break;  // no data
        *instrp >> opiHdr[i];
    }
    if(instrp->atEnd())
    {
        infilep->close();
        delete instrp;
        delete infilep;
        return -1;
    }

    if(!((opiHdr[11] != 0x4F) || (opiHdr[12] != 0x50) || (opiHdr[13] != 0x49)
            || (opiHdr[14] != 0x55) || (opiHdr[15] != 0x43) || (opiHdr[16] != 0x45)))
    {
       infilep->close();
        delete instrp;
        delete infilep;
        return 1;
    }

    infilep->reset();
    // check if it is edf
    if(edfhdrread(instrp, &lpid, &lrid, &startDT, &numDataRecs, &dataRecDur,
                  &numSignals, &labelSignalsQV, &transTypeQV, &physDimQV, &physMinQV,
                  &physMaxQV, &digMinQV, &digMaxQV, &prefiltQV, &sampsPerDRQV) == 0)
    {
        // check if generic opi data
        if((numSignals == 7) && (dataRecDur == EDFDRDURSEC) &&
                (sampsPerDRQV.at(0) == ADCLEN*FRMSPERSEC*EDFDRDURSEC) &&
                (sampsPerDRQV.at(1) == 1*FRMSPERSEC*EDFDRDURSEC) &&
                (sampsPerDRQV.at(2) == 1*FRMSPERSEC*EDFDRDURSEC) &&
                (sampsPerDRQV.at(3) == ACCLEN*FRMSPERSEC*EDFDRDURSEC) &&
                (sampsPerDRQV.at(4) == 1*FRMSPERSEC*EDFDRDURSEC) &&
                (sampsPerDRQV.at(5) == 1*FRMSPERSEC*EDFDRDURSEC) &&
                (sampsPerDRQV.at(6) == 30))
        {
            infilep->close();
            delete instrp;
            delete infilep;
            return EDFEFILETYPE;
        }
    }
    // cleanup
    infilep->close();
    delete instrp;
    delete infilep;

    return UNKNOWNFILETYPE;   // if here, didn't pass any recognized tests
}


/***
  * Delete qvectors
  */
void delQVs(QVector<qint64> **tsQVpp, QVector<quint8> **skpQVpp,
            QVector<quint8> **batQVpp, QVector<qint16> **adcQVpp,
            QVector<qint16> **tmpQVpp, QVector<qint16> **axQVpp,
            QVector<qint16> **ayQVpp, QVector<qint16> **azQVpp,
            QVector<qint16> **sqQVpp, QVector<quint8> **edQVpp)
{
    qint32 pdnSlot;

    for(pdnSlot = 0; pdnSlot < PDNLISTLEN; pdnSlot++)
    {
        if(tsQVpp[pdnSlot] != 0)
        {
            delete tsQVpp[pdnSlot];
            tsQVpp[pdnSlot] = 0;
        }
        if(skpQVpp[pdnSlot] != 0)
        {
            delete skpQVpp[pdnSlot];
            skpQVpp[pdnSlot] = 0;
        }
        if(batQVpp[pdnSlot] != 0)
        {
            delete batQVpp[pdnSlot];
            batQVpp[pdnSlot] = 0;
        }
        if(adcQVpp[pdnSlot] != 0)
        {
            delete adcQVpp[pdnSlot];
            adcQVpp[pdnSlot] = 0;
        }
        if(tmpQVpp[pdnSlot] != 0)
        {
            delete tmpQVpp[pdnSlot];
            tmpQVpp[pdnSlot] = 0;
        }
        if(axQVpp[pdnSlot] != 0)
        {
            delete axQVpp[pdnSlot];
            axQVpp[pdnSlot] = 0;
        }
        if(ayQVpp[pdnSlot] != 0)
        {
            delete ayQVpp[pdnSlot];
            ayQVpp[pdnSlot] = 0;
        }
        if(azQVpp[pdnSlot] != 0)
        {
            delete azQVpp[pdnSlot];
            azQVpp[pdnSlot] = 0;
        }
        if(sqQVpp[pdnSlot] != 0)
        {
            delete sqQVpp[pdnSlot];
            sqQVpp[pdnSlot] = 0;
        }
        if(edQVpp[pdnSlot] != 0)
        {
            delete edQVpp[pdnSlot];
            edQVpp[pdnSlot] = 0;
        }
    }
}


/***
  * Delete sepcific set of qvectors
  */
void delspecQVs(qint32 pdnSlot,
                               QVector<qint64> **tsQVpp, QVector<quint8> **skpQVpp,
                               QVector<quint8> **batQVpp, QVector<qint16> **adcQVpp,
                               QVector<qint16> **tmpQVpp, QVector<qint16> **axQVpp,
                               QVector<qint16> **ayQVpp, QVector<qint16> **azQVpp,
                               QVector<qint16> **sqQVpp, QVector<quint8> **edQVpp)
{
    if(tsQVpp[pdnSlot] != 0)
    {
        delete tsQVpp[pdnSlot];
        tsQVpp[pdnSlot] = 0;
    }
    if(skpQVpp[pdnSlot] != 0)
    {
        delete skpQVpp[pdnSlot];
        skpQVpp[pdnSlot] = 0;
    }
    if(batQVpp[pdnSlot] != 0)
    {
        delete batQVpp[pdnSlot];
        batQVpp[pdnSlot] = 0;
    }
    if(adcQVpp[pdnSlot] != 0)
    {
        delete adcQVpp[pdnSlot];
        adcQVpp[pdnSlot] = 0;
    }
    if(tmpQVpp[pdnSlot] != 0)
    {
        delete tmpQVpp[pdnSlot];
        tmpQVpp[pdnSlot] = 0;
    }
    if(axQVpp[pdnSlot] != 0)
    {
        delete axQVpp[pdnSlot];
        axQVpp[pdnSlot] = 0;
    }
    if(ayQVpp[pdnSlot] != 0)
    {
        delete ayQVpp[pdnSlot];
        ayQVpp[pdnSlot] = 0;
    }
    if(azQVpp[pdnSlot] != 0)
    {
        delete azQVpp[pdnSlot];
        azQVpp[pdnSlot] = 0;
    }
    if(sqQVpp[pdnSlot] != 0)
    {
        delete sqQVpp[pdnSlot];
        sqQVpp[pdnSlot] = 0;
    }
    if(edQVpp[pdnSlot] != 0)
    {
        delete edQVpp[pdnSlot];
        edQVpp[pdnSlot] = 0;
    }
}

/***
  * Read EDF header information into pointed to qvectors.
  * Returns:
  *     -1, if error
  *     0, if successful
  */
qint32 edfhdrread(QDataStream *instrp, QString *lpidp, QString *lridp,
                                 QDateTime *startDTp, qint32 *numDataRecsp, qint32 *dataRecDurp,
                                 qint32 *numSignalsp, QVector<QString> *labelSignalsQVp,
                                 QVector<QString> *transTypeQVp, QVector<QString> *physDimQVp,
                                 QVector<qint32> *physMinQVp, QVector<qint32> *physMaxQVp,
                                 QVector<qint32> *digMinQVp, QVector<qint32> *digMaxQVp,
                                 QVector<QString> *prefiltQVp, QVector<qint32> *sampsPerDRQVp)
{
    qint32 i, hdrBytes;
    char buff[100];

    //8 ascii : version of this data format (0)
    if(instrp->readRawData(buff,8) < 0) return -1;
    if(QString::compare(QString::fromAscii(buff,8), "0       ")) return -1;

    //80 ascii : local patient identification
    if(instrp->readRawData(buff,80) < 0) return -1;
    *lpidp = QString::fromAscii(buff,80);

    //80 ascii : local recording identification.
    if(instrp->readRawData(buff,80) < 0) return -1;
    *lridp = QString::fromAscii(buff,80);

    //8 ascii : startdate of recording (dd.mm.yy),
    //+8 ascii : starttime of recording (hh.mm.ss).
    if(instrp->readRawData(buff,16) < 0) return -1;
    *startDTp = QDateTime::fromString(QString::fromAscii(buff,16),"dd.MM.yyHH.mm.ss");
    if(*startDTp < QDateTime::fromString("19850101","yyyyMMdd")) // if yy=13 should be 2013
        *startDTp = startDTp->addYears(100);

    //8 ascii : number of bytes in header record
    if(instrp->readRawData(buff,8) < 0) return -1;
    hdrBytes = QString::fromAscii(buff,8).toInt();

    //44 ascii : reserved
    if(instrp->readRawData(buff,44) < 0) return -1;
    if(QString::compare(QString::fromAscii(buff,44), QString("EDF+C").leftJustified(44,' ')))
        return -1;

    //8 ascii : number of data records (-1 if unknown)
    if(instrp->readRawData(buff,8) < 0) return -1;
    *numDataRecsp = QString::fromAscii(buff,8).toInt();

    //8 ascii : duration of a data record, in seconds
    if(instrp->readRawData(buff,8) < 0) return -1;
    *dataRecDurp = QString::fromAscii(buff,8).toInt();

    //4 ascii : number of signals (ns) in data record
    if(instrp->readRawData(buff,4) < 0) return -1;
    *numSignalsp = QString::fromAscii(buff,4).toInt();

    //ns * 16 ascii : ns * label
    for(i = 0; i < (*numSignalsp); i++)
    {
        if(instrp->readRawData(buff,16) < 0) return -1;
        labelSignalsQVp->append(QString::fromAscii(buff,16));
    }

    //ns * 80 ascii : ns * transducer type (e.g. AgAgCl electrode)
    for(i = 0; i < (*numSignalsp); i++)
    {
        if(instrp->readRawData(buff,80) < 0) return -1;
        transTypeQVp->append(QString::fromAscii(buff,80));
    }

    //ns * 8 ascii : ns * physical dimension (e.g. uV)
    for(i = 0; i < (*numSignalsp); i++)
    {
        if(instrp->readRawData(buff,8) < 0) return -1;
        physDimQVp->append(QString::fromAscii(buff,8));
    }

    //ns * 8 ascii : ns * physical minimum (e.g. -500 or 34)
    for(i = 0; i < (*numSignalsp); i++)
    {
        if(instrp->readRawData(buff,8) < 0) return -1;
        physMinQVp->append(QString::fromAscii(buff,8).toInt());
    }

    //ns * 8 ascii : ns * physical maximum (e.g. 500 or 40)
    for(i = 0; i < (*numSignalsp); i++)
    {
        if(instrp->readRawData(buff,8) < 0) return -1;
        physMaxQVp->append(QString::fromAscii(buff,8).toInt());
    }

    //ns * 8 ascii : ns * digital minimum (e.g. -2048)
    for(i = 0; i < (*numSignalsp); i++)
    {
        if(instrp->readRawData(buff,8) < 0) return -1;
        digMinQVp->append(QString::fromAscii(buff,8).toInt());
    }

    //ns * 8 ascii : ns * digital maximum (e.g. 2047)
    for(i = 0; i < (*numSignalsp); i++)
    {
        if(instrp->readRawData(buff,8) < 0) return -1;
        digMaxQVp->append(QString::fromAscii(buff,8).toInt());
    }

    //ns * 80 ascii : ns * prefiltering (e.g. HP:0.1Hz LP:75Hz)
    for(i = 0; i < (*numSignalsp); i++)
    {
        if(instrp->readRawData(buff,80) < 0) return -1;
        prefiltQVp->append(QString::fromAscii(buff,8));
    }

    //ns * 8 ascii : ns * nr of samples in each data record
    for(i = 0; i < (*numSignalsp); i++)
    {
        if(instrp->readRawData(buff,8) < 0) return -1;
        sampsPerDRQVp->append(QString::fromAscii(buff,8).toInt());
    }

    //ns * 32 ascii : ns * reserved
    for(i = 0; i < (*numSignalsp); i++)
        if(instrp->readRawData(buff,32) < 0) return -1;

    return 0;   // if got here, then read all of edf header
}


/***
  * Write EDF header especially for opi_e raw data files, 8 sec data record duration
  * Inputs:
  *     out, ptr to output data stream
  *     lpidp, ptr to local patient id
  *     lridp, ptr to local recording id
  *     startDTp, ptr to starting date and time
  *     numDataRecs, number of data records
  * Returns:
  *     true, if successful
  *     false, if not successful
  */
void edfEhdropiwrite(QDataStream *out, QString *lpidp, QString *lridp,
                     QDateTime *startDTp, qint32 numDataRecs)
{
    QString tempstr;

    out->writeRawData("0       ", 8);     // edf version of data format
    out->writeRawData(lpidp->toUtf8().leftJustified(80,' ').data(),80);   // local patient identification
    out->writeRawData(lridp->toUtf8().leftJustified(80,' ').data(),80);   // local recording identification
    out->writeRawData(startDTp->toString("dd.MM.yyhh.mm.ss").toUtf8().data(),16); // startdate and starttime
    out->writeRawData("2048    ", 8);     // number of header bytes (256+7signals*256)
    out->writeRawData(QByteArray("EDF+C").leftJustified(44,' ').data(),44); // format type (reserved)
    out->writeRawData(QString("%1").arg(numDataRecs).toUtf8().leftJustified(8,' ').data(),8);  // number of data records
    out->writeRawData("8       ", 8);     // duration of a data record in seconds
    out->writeRawData("7   ", 4);     // number of signals: adc, accX, accY, accZ, temp, EDF annotations

    // signal labels
    out->writeRawData("ADC             ", 16);   // maybe change if know type later on
    out->writeRawData("Accel. X-axis   ", 16);
    out->writeRawData("Accel. Y-axis   ", 16);
    out->writeRawData("Accel. Z-axis   ", 16);
    out->writeRawData("Temperature     ", 16);
    out->writeRawData("Activity        ", 16);
    out->writeRawData("EDF Annotations ", 16);

    // transducer type
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);

    // physical dimensions
    out->writeRawData("uV      ", 8);
    out->writeRawData("g       ", 8);
    out->writeRawData("g       ", 8);
    out->writeRawData("g       ", 8);
    out->writeRawData("degreeC ", 8);
    out->writeRawData("dB      ", 8);
    out->writeRawData("        ", 8);

    // physical mins and maxs
    out->writeRawData("-800    ", 8);
    out->writeRawData("-2      ", 8);
    out->writeRawData("-2      ", 8);
    out->writeRawData("-2      ", 8);
    out->writeRawData("-47     ", 8);
    out->writeRawData("-50     ", 8);
    out->writeRawData("-1      ", 8);

    out->writeRawData("800     ", 8);
    out->writeRawData("2       ", 8);
    out->writeRawData("2       ", 8);
    out->writeRawData("2       ", 8);
    out->writeRawData("241     ", 8);
    out->writeRawData("50      ", 8);
    out->writeRawData("1       ", 8);

    // digital mins and maxs
    out->writeRawData("-20480  ", 8);
    out->writeRawData("-32768  ", 8);
    out->writeRawData("-32768  ", 8);
    out->writeRawData("-32768  ", 8);
    out->writeRawData("0       ", 8);
    out->writeRawData("-32768   ", 8);
    out->writeRawData("-32768  ", 8);

    out->writeRawData("20480   ", 8);
    out->writeRawData("32767   ", 8);
    out->writeRawData("32767   ", 8);
    out->writeRawData("32767   ", 8);
    out->writeRawData("4080    ", 8);
    out->writeRawData("32767   ", 8);
    out->writeRawData("32767   ", 8);

    // prefiltering
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);
    out->writeRawData(QByteArray(" ").leftJustified(80,' ').data(),80);

    // number of samples in each data record (8s)
    out->writeRawData("2048    ", 8);
    out->writeRawData("32      ", 8);
    out->writeRawData("32      ", 8);
    out->writeRawData("128     ", 8);
    out->writeRawData("32      ", 8);
    out->writeRawData("32      ", 8);
    out->writeRawData("30      ", 8);

    // reserved fields
    out->writeRawData(QByteArray(" ").leftJustified(32,' ').data(),32);
    out->writeRawData(QByteArray(" ").leftJustified(32,' ').data(),32);
    out->writeRawData(QByteArray(" ").leftJustified(32,' ').data(),32);
    out->writeRawData(QByteArray(" ").leftJustified(32,' ').data(),32);
    out->writeRawData(QByteArray(" ").leftJustified(32,' ').data(),32);
    out->writeRawData(QByteArray(" ").leftJustified(32,' ').data(),32);
    out->writeRawData(QByteArray(" ").leftJustified(32,' ').data(),32);
}


/***
  * Need to know beforedhand knowledge how data is formatted in different streams
  * Will alter temperature stream for averaging so need that flag
  * Returns number of data records written according to opi EDF format
  */
qint32 edfEwrite(QDataStream *outstrp, QVector<qint16> *adcQVp,
                 QVector<qint16> *tmpQVp, QVector<qint16> *axQVp,
                 QVector<qint16> *ayQVp, QVector<qint16> *azQVp,
                 QVector<qint16> *sqQVp, qint32 startDataRecCt,
                 qint64 firstFrmTS, QVector<qint64> *tsQVp,
                 QVector<qint64> *tagTSQVp, QVector<QString> *tagTextQVp)
{
    bool noMore;    // indicate if no more data to write into data records
    qint32 i, j, dataRecordCt;
    QProgressDialog progQPD("Writing EDF file",QString(),0,azQVp->size()/(ACCLEN*FRMSPERSEC*EDFDRDURSEC));
    QByteArray tempQBA;
    progQPD.setWindowModality(Qt::WindowModal);
    progQPD.setMinimumDuration(3000);
    QString tempQS;

    // Write EDF in data record segments
    noMore = false;
    dataRecordCt = 0;

    for(j = 0; j < tmpQVp->size()/(FRMSPERSEC*EDFDRDURSEC); j++)
    {
        if((dataRecordCt % 10) == 0)
        {
            progQPD.setValue(dataRecordCt);
            qApp->processEvents();
        }
        // make sure there is enough data for another data record, otherwise get out
        if((((j+1)*ADCLEN*FRMSPERSEC*EDFDRDURSEC-1) > adcQVp->size()) ||
                (((j+1)*TMPLEN*FRMSPERSEC*EDFDRDURSEC-1) > tmpQVp->size()) ||
                (((j+1)*ACCLEN/4*FRMSPERSEC*EDFDRDURSEC-1) > axQVp->size()) ||
                (((j+1)*ACCLEN/4*FRMSPERSEC*EDFDRDURSEC-1) > ayQVp->size()) ||
                (((j+1)*ACCLEN*FRMSPERSEC*EDFDRDURSEC-1) > azQVp->size()) ||
                (((j+1)*1*FRMSPERSEC*EDFDRDURSEC-1) > sqQVp->size()))
            break;
        for(i = 0; i < ADCLEN*FRMSPERSEC*EDFDRDURSEC; i++)
            *outstrp << adcQVp->at(j*ADCLEN*FRMSPERSEC*EDFDRDURSEC+i);
        for(i = 0; i < ACCLEN/4*FRMSPERSEC*EDFDRDURSEC; i++)
            *outstrp << axQVp->at(j*ACCLEN/4*FRMSPERSEC*EDFDRDURSEC+i);
        for(i = 0; i < ACCLEN/4*FRMSPERSEC*EDFDRDURSEC; i++)
            *outstrp << ayQVp->at(j*ACCLEN/4*FRMSPERSEC*EDFDRDURSEC+i);
        for(i = 0; i < ACCLEN*FRMSPERSEC*EDFDRDURSEC; i++)
            *outstrp << azQVp->at(j*ACCLEN*FRMSPERSEC*EDFDRDURSEC+i);
        for(i = 0; i < TMPLEN*FRMSPERSEC*EDFDRDURSEC; i++)
            *outstrp << tmpQVp->at(j*TMPLEN*FRMSPERSEC*EDFDRDURSEC+i);
        for(i = 0; i < 1*FRMSPERSEC*EDFDRDURSEC; i++)
        {
            *outstrp << sqQVp->at(j*1*FRMSPERSEC*EDFDRDURSEC+i);
        }
        // EDF Annotations, max of 2 annotations per data record, each with 52 total chars
        tempQBA.clear();
        tempQBA.append(QString("+%1").arg((dataRecordCt+startDataRecCt)*EDFDRDURSEC).toUtf8().append(QChar(20)).append(QChar(20)).append(QChar(0)));
        for(i = 0; i < 2; i++)
        {
            if((tagTSQVp->size() > 0) && (tagTSQVp->at(0) <= (tsQVp->at(j*1*FRMSPERSEC*EDFDRDURSEC) + 1*FRMSPERSEC*EDFDRDURSEC*ADCLEN*UCERTCFREQ/TSERTCFREQ)))
            {
                tempQS = tagTextQVp->at(0);
                tempQS.truncate(14);    // limit text to 14 charcters
                tempQBA.append(QString("+%1").arg((float)(tagTSQVp->at(0)-firstFrmTS)/UCERTCFREQ).toUtf8().append(QChar(20)).append(tempQS.toUtf8()));
                tempQBA.append(QChar(20)).append(QChar(0));
                tagTSQVp->remove(0,1);
                tagTextQVp->remove(0,1);
            }
        }
        outstrp->writeRawData(tempQBA.leftJustified(60,0x00), 60);
        dataRecordCt++;
    }
    return dataRecordCt;
}


/***
  * Process frames by inserting missing frames if needed
  */
void procQV(QVector<qint64> *tsQVp, QVector<quint8> *skpQVp,
            QVector<quint8> *batQVp, QVector<qint16> *adcQVp,
            QVector<qint16> *tmpQVp, QVector<qint16> *axQVp,
            QVector<qint16> *ayQVp, QVector<qint16> *azQVp,
            QVector<qint16> *sqQVp, QVector<quint8> *edQVp)
{
    qint32 i, j;
    qint32 currADCIndex;  // need to keep track since sometimes there is 62 data (usually 64)
    double cntr2FrmRatioSet, delFrmCt;
    qint32 missFrmCt;

    // Set the counter to frame ratio
    cntr2FrmRatioSet = ((double) (ADCLEN*UCERTCFREQ))/((double) (TSERTCFREQ));

    // process all the frames backwards since inserting data into vector
    if(skpQVp->at(skpQVp->size()-1) == 0)
        currADCIndex = adcQVp->size()-ADCLEN;
    else    // skipped sample
        currADCIndex = adcQVp->size()-(ADCLEN-2);
    for(i = tsQVp->size()-1; i > 0; i--)
    {
        delFrmCt = ((double) (tsQVp->at(i)-tsQVp->at(i-1)))/cntr2FrmRatioSet;
        missFrmCt = ((qint32) (delFrmCt + 0.5))-1;	// for rounding

        if((missFrmCt > 0) && (missFrmCt < MAXMISSFRMS))  // only fill positive frames and that less than a number
        // qvector should only be filled with monotonically increasing time
        {
            qDebug() << "adding missing frame" << missFrmCt << tsQVp->at(i) << tsQVp->at(i-1) << i << tsQVp->size() << adcQVp->size() << tmpQVp->size() << axQVp->size() << azQVp->size();
            // insert missing timeslots in array
            tsQVp->insert(i, missFrmCt, tsQVp->at(i-1)+((qint64)cntr2FrmRatioSet));
            for(j = 1; j < missFrmCt; j++)
            {
                tsQVp->replace(i+j, tsQVp->at(i+j-1)+((qint64)cntr2FrmRatioSet));
            }
            skpQVp->insert(i, missFrmCt, 0);
            batQVp->insert(i, missFrmCt, 1);
            adcQVp->insert(currADCIndex, missFrmCt*ADCLEN, 0);
            tmpQVp->insert(i, missFrmCt, tmpQVp->at(i-1));
            axQVp->insert(i, missFrmCt, axQVp->at(i-1));
            ayQVp->insert(i, missFrmCt, ayQVp->at(i-1));
            azQVp->insert(i*ACCLEN, missFrmCt*ACCLEN, azQVp->at(i*ACCLEN-1));
            sqQVp->insert(i, missFrmCt, -4000); //4 for missing pkt
            edQVp->insert(i, missFrmCt, 0);
        }
        else if(missFrmCt != 0)
        {
            qDebug() << "invalid missing frame count to add, ignoring" << missFrmCt;
        }
        if(skpQVp->at(i-1) == 0)
            currADCIndex -= ADCLEN;
        else
            currADCIndex -= (ADCLEN-2);
    }
    if(i > 0)
    {
        tsQVp->remove(0, i);
        skpQVp->remove(0, i);
        batQVp->remove(0, i);
        adcQVp->remove(0, currADCIndex);
        tmpQVp->remove(0, i);
        axQVp->remove(0, i);
        ayQVp->remove(0, i);
        azQVp->remove(0, i*ACCLEN);
        sqQVp->remove(0, i);
        edQVp->remove(0, i);
    }
}

/***
  * Process frames by inserting missing frames if needed and
  * removing start and end frames
  */
/*
void procQV(qint64 *firstFrmTSp, qint64 *lastFrmTSp,
            QVector<qint64> **tsQVpp, QVector<quint8> **skpQVpp,
            QVector<quint8> **batQVpp, QVector<qint16> **adcQVpp,
            QVector<qint16> **tmpQVpp, QVector<qint16> **axQVpp,
            QVector<qint16> **ayQVpp, QVector<qint16> **azQVpp,
            QVector<qint16> **sqQVpp, QVector<quint8> **edQVpp)
{
    qint32 pdnSlot, i, j;
    qint32 currADCIndex;  // need to keep track since sometimes there is 62 data (usually 64)
    double cntr2FrmRatioSet, delFrmCt;
    qint32 missFrmCt;

    // Set the counter to frame ratio
    cntr2FrmRatioSet = ((double) (64*UCRTCFREQ))/((double) (TSRTCFREQ));

    for(pdnSlot = 0; pdnSlot < PDNLISTLEN; pdnSlot++)
    {
        if(tsQVpp[pdnSlot] == 0) continue;
        if(tsQVpp[pdnSlot]->size() < 2) continue;

        // process all the frames backwards since inserting data into vector
        if(skpQVpp[pdnSlot]->at(skpQVpp[pdnSlot]->size()-1) == 0)
            currADCIndex = adcQVpp[pdnSlot]->size()-ADCLEN;
        else    // skipped sample
            currADCIndex = adcQVpp[pdnSlot]->size()-(ADCLEN-2);
        for(i = tsQVpp[pdnSlot]->size()-1; i > 0; i--)
        {
            // remove frames at end, have to do one by one because must examine
            // if frame had 62 or 64 adc samples
            if(tsQVpp[pdnSlot]->at(i) > (*lastFrmTSp + 4))
            {
                tsQVpp[pdnSlot]->remove(i);
                skpQVpp[pdnSlot]->remove(i);
                batQVpp[pdnSlot]->remove(i);
                adcQVpp[pdnSlot]->remove(currADCIndex, adcQVpp[pdnSlot]->size()-currADCIndex);
                if(skpQVpp[pdnSlot]->at(i-1) == 0)
                    currADCIndex -= ADCLEN;
                else
                    currADCIndex -= (ADCLEN-2);
                tmpQVpp[pdnSlot]->remove(i);
                axQVpp[pdnSlot]->remove(i);
                ayQVpp[pdnSlot]->remove(i);
                azQVpp[pdnSlot]->remove(i*ACCLEN, ACCLEN);
                sqQVpp[pdnSlot]->remove(i);
                edQVpp[pdnSlot]->remove(i);
                continue;
            }

            // get out to remove frames in beginning, add fudge factor of 4
            // for rounding issues
            if(tsQVpp[pdnSlot]->at(i) < (*firstFrmTSp - 4)) break;
            if(i > 0) // check previous one too so that we don't add lots of missing frames
            {
                if(tsQVpp[pdnSlot]->at(i-1) <= (*firstFrmTSp - 4)) break;
            }

            delFrmCt = ((double) (tsQVpp[pdnSlot]->at(i)-tsQVpp[pdnSlot]->at(i-1)))/cntr2FrmRatioSet;
            missFrmCt = ((qint32) (delFrmCt + 0.5))-1;	// for rounding

            if(missFrmCt > 0)  // only fill positive frames, shouldn't be negative since
                               // qvector should only be filled with monotonically increasing time
            {
                //qDebug() << "Missing Frames" << missFrmCt;
                if(missFrmCt > 345600) missFrmCt = 345600; // max is half a day addition
                // insert missing timeslots in array
                tsQVpp[pdnSlot]->insert(i, missFrmCt, tsQVpp[pdnSlot]->at(i-1)+((qint64)cntr2FrmRatioSet));
                for(j = 1; j < missFrmCt; j++)
                {
                    tsQVpp[pdnSlot]->replace(i+j, tsQVpp[pdnSlot]->at(i+j-1)+((qint64)cntr2FrmRatioSet));
                }
                skpQVpp[pdnSlot]->insert(i, missFrmCt, 0);
                batQVpp[pdnSlot]->insert(i, missFrmCt, 1);
                adcQVpp[pdnSlot]->insert(currADCIndex, missFrmCt*ADCLEN, 0);
                tmpQVpp[pdnSlot]->insert(i, missFrmCt, tmpQVpp[pdnSlot]->at(i-1));
                axQVpp[pdnSlot]->insert(i, missFrmCt, axQVpp[pdnSlot]->at(i-1));
                ayQVpp[pdnSlot]->insert(i, missFrmCt, ayQVpp[pdnSlot]->at(i-1));
                azQVpp[pdnSlot]->insert(i*ACCLEN, missFrmCt*ACCLEN, azQVpp[pdnSlot]->at(i*ACCLEN-1));
                sqQVpp[pdnSlot]->insert(i, missFrmCt, sqQVpp[pdnSlot]->at(i-1));
                edQVpp[pdnSlot]->insert(i, missFrmCt, 0);
            }
            if(skpQVpp[pdnSlot]->at(i-1) == 0)
                currADCIndex -= ADCLEN;
            else
                currADCIndex -= (ADCLEN-2);
        }
        if(i > 0)
        {
            tsQVpp[pdnSlot]->remove(0, i);
            skpQVpp[pdnSlot]->remove(0, i);
            batQVpp[pdnSlot]->remove(0, i);
            adcQVpp[pdnSlot]->remove(0, currADCIndex);
            tmpQVpp[pdnSlot]->remove(0, i);
            axQVpp[pdnSlot]->remove(0, i);
            ayQVpp[pdnSlot]->remove(0, i);
            azQVpp[pdnSlot]->remove(0, i*ACCLEN);
            sqQVpp[pdnSlot]->remove(0, i);
            edQVpp[pdnSlot]->remove(0, i);
        }
    }
}
*/

// Returns a QString in format of "UTC+HH:mm" with the offset of local time
// from UTC contained in +HH:mm. This function depends on the system time.
// granularity is 15 minutes
QString localUTCOffset(void)
{
    qint64 diffMSecs, diffMins;
    QString retQS;

    diffMSecs = QDateTime::fromString(QDateTime::currentDateTime().toString("yyyyMMddhhmmsszzz"),"yyyyMMddhhmmsszzz").toMSecsSinceEpoch()
            - QDateTime::fromString(QDateTime::currentDateTimeUtc().toString("yyyyMMddhhmmsszzz"),"yyyyMMddhhmmsszzz").toMSecsSinceEpoch();
    diffMins = diffMSecs/60000;
    retQS = QString("UTC");
    if(diffMins < 0)
    {
        retQS.append("-");
        diffMins *= -1;
    }
    else
    {
        retQS.append("+");
    }
    diffMins = (diffMins+7)/15*15;  // Round to nearest 15 minutes
    retQS.append(QString("%1:%2").arg(diffMins/60,2,10,QChar('0')).arg(diffMins%60,2,10,QChar('0')));

    return retQS;
}


/***
  *	Use the Controller to take 100 measurements of current ZigBee Channel
  *  returning the maximum peak Energy Detected.
  *  Assumes the comport has already been opened by SDK.
  *	Inputs:
  *		comportptr, pointer to handle
  *	Returns:
  *      non-negative integer representing maximum peak Energy Detected
  *      -1, if error
  */
qint32 maxWLMeasure100(HANDLE *comportptr)
{
    OPIPKT_t opipkt;
    qint32 i;
    qint32 maxED;

    maxED = 0;
    for(i = 0; i < 100; i++)
    {
        if(opiuce_wlmeasure(comportptr, &opipkt))
        {
            return -1;
        }
        if(opipkt.payload[2] > maxED) maxED = opipkt.payload[2];
    }
    return maxED;
}
