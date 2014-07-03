#include "relax.h"
#include "ui_relax.h"


ReLax::ReLax(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ReLax)
{
    qint32 i;

    ui->setupUi(this);

    // limit screen size to current
    this->setMinimumSize(this->width(),430);
    this->setMaximumSize(this->width(),430);
    // initialize variables
    // set to on mode if connected
    comPort = 0;
    timerEventCt = 0;
    noPktTimerCt = 0;
    // stuff data into queue for FFT according to overlap
    for(i = 0; i < LIVEFFTLENOVERLAP; i++)
        signalDataToFFTQQ.enqueue(0);
    // set initial state variables
    lCalc = 0;
    bCalc = 0;
    gCalc = 0;
    mCalc = 0;
    gmCalc = 0;
    bmCalc = 0;
    stRelax_now = LEDNON;
    stRelax_prev = LEDNON;
    stRelax_pktCt = 0;
    stRelax_accum = 0;
    myQFT.setSize(LIVEFFTLEN);
    myQFT.setWindowFunction("Hann");
    tsepdn = 0xFF;  // unknown here

    configDoneFlag = false;
    outQFp = 0;
    outQDSp = 0;
    prevFrmTS = 0;
    firstFrmTS = 0;
    newstep = 0;
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

    TDVp = NULL;

    // do configuration checks and settings
    qqscenep = new qqwidget(this,ui->mainfigSW->widget(SAVEDMODE)->width(),ui->mainfigSW->widget(SAVEDMODE)->height());
    ui->gridLayout->addWidget(qqscenep,0,0);
    signalQGSp = new QGraphicsScene(ui->signalGV->x(),ui->signalGV->y(),ui->signalGV->width()-2,ui->signalGV->height()-2,this);
    ui->signalGV->setScene(signalQGSp);
    spectQGSp= new QGraphicsScene(ui->spectGV->x(),ui->spectGV->y(),ui->spectGV->width()-2,ui->spectGV->height()-2,this);
    ui->spectGV->setScene(spectQGSp);
    ledQGSp = new QGraphicsScene(ui->ledGV->x(),ui->ledGV->y(),ui->ledGV->width()-2,ui->ledGV->height()-2,this);
    ui->ledGV->setScene(ledQGSp);
    signalZoomRatio = SIGNALZOOMRATIOINIT;
    // have a fade in whenever we change modes
    connect(ui->configSW, SIGNAL(currentChanged(int)), this, SLOT(fadeInWidget(int)));
    // start timer since start in LIVEMODE
    timer.start(UPDATEPERIOD, this);
}

void ReLax::dragEnterEvent(QDragEnterEvent *e)
{
    if(ui->modeSli->value()==SAVEDMODE)
    {

        if(e->mimeData()->hasFormat("text/uri-list"))
        {
            QList<QUrl> urls = e->mimeData()->urls();

            if(urls.isEmpty())
            {
                return;
            }
            else
            {

                QString filename = urls[0].toLocalFile();
                if(ui->savedFilenameLE->text()==filename)
                    return;
                if(filename.isEmpty())
                    return;
                if(!filename.contains("edf"))
                {
                    QMessageBox msg;
                    msg.setText("please drag .edf file in here");
                    msg.exec();
                    return ;
                }
                ui->savedFilenameLE->setText(filename);
                QMessageBox msg;
                msg.setText("load "+filename);
                msg.exec();
                return ;
            }
        }
    }
}


ReLax::~ReLax()
{
    delete ui;
}


void ReLax::fadeInWidget(int index)
{
    QPointer<FaderWidget> faderWidget;
    if (faderWidget)
        faderWidget->close();
    faderWidget = new FaderWidget(ui->configSW->widget(index));
    faderWidget->start();
}


// used for LIVEMODE
void ReLax::timerEvent(QTimerEvent *event)
{
    qint32 i, k;
    OPIPKT_t ucOpipkt, tseDataOpipkt, tseFFTOpipkt;
    quint8 ucusdStatus, mytsepdn;
    qint64 tsedsn;
    qint16 adcValue;
    qint8 sampQual = 0;
    int getDataRes;
    float timeData[LIVEFFTLEN], freqData[LIVEFFTLEN];
    QVector<QComplexFloat> freqQCFQV;
    double tempDoub;
    qint64 frmTS;
    QVector<qint64> tagTSQV;
    QVector<QString> tagTextQV;
    double beginOffFrms, beginOffFrmsj;
    qint32 beginOffFrmsi;
    qint32 dataRecCt;
    QMessageBox msgQMB;
    qint32 currMonth;
    QPainterPath *qp;
    bool liveEnableFlg;
    QTime killTime;
    qint32 resWLScan[ZBWLCHANCT];
    qint32 resMinInd, bestzbChan;
    QProgressDialog progQPD("Configuring", QString(), 0, ZBWLCHANCT);
    bool configFailFlg;
    quint16 tempui16;
    float rlevelCalc;

    // initialize data in case there are errors later
    ucOpipkt.payload[DSNLEN+TSLEN+6+FWVLEN+1+PDNLISTLEN+1] = 0;

    // start
    if(event->timerId() == timer.timerId())
    {
        // check if device is in every 3 sec (90*UPDATEPERIOD) and not being used
        if(((timerEventCt % 90) == 0) && (comPort == 0))
        {
            configFailFlg = false;
            progQPD.setWindowModality(Qt::WindowModal);
            progQPD.setMinimumDuration(10); // force it to open

            // do configuration once if ts is plugged in, otherwise skip
            if(!configDoneFlag && (comPort == 0))
            {
                if(opi_openuce_com(&comPort))   // open com port, should open unless device was removed
                {
                    opi_closeuce_com(&comPort);
                    comPort = 0;                    
                }
                if(comPort != 0)
                {
                    if(opiuce_onmode(&comPort))
                    {
                        killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                        while(QTime::currentTime() < killTime);
                        if(opiuce_onmode(&comPort))     // try again
                        {
                            opi_closeuce_com(&comPort);
                            comPort = 0;
                            configFailFlg = true;
                        }
                    }
                    killTime = QTime::currentTime().addMSecs(1100);
                    while(QTime::currentTime() < killTime); // wait so that ts has a chance to startup

                    opiuce_evcaperase(&comPort); // erase events/tags

                    if(opiuce_status(&comPort, &ucOpipkt))
                    {
                        killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                        while(QTime::currentTime() < killTime);
                        if(opiuce_status(&comPort, &ucOpipkt))
                        {
                            opi_closeuce_com(&comPort);
                            comPort = 0;
                            configFailFlg = true;
                        }
                    }

                    ucusdStatus = ucOpipkt.payload[DSNLEN+TSLEN+6+FWVLEN+1+PDNLISTLEN+1];
                    if(ucusdStatus & 0x01)  // if ts in
                    {
                        // erase settings
                        if(opiuce_forgettsesettings(&comPort, 0))
                        {
                            killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                            while(QTime::currentTime() < killTime);
                            if(opiuce_forgettsesettings(&comPort, 0))
                            {
                                opi_closeuce_com(&comPort);
                                comPort = 0;
                                configFailFlg = true;
                            }
                        }
                        // default tse to regular settings
                        if(opiuce_tsestatus(&comPort, &ucOpipkt))
                        {
                            opi_closeuce_com(&comPort);
                            comPort = 0;
                            configFailFlg = true;
                        }
                        // pdn can never be 255, will cause problems with controller
                        mytsepdn = ucOpipkt.payload[1+DSNLEN+5+FWVLEN];
                        if(mytsepdn == 255)
                        {
                            tsedsn = ((qint64) ucOpipkt.payload[1] << 32) + ((qint64) ucOpipkt.payload[2] << 24)
                                    + (ucOpipkt.payload[3] << 16) + (ucOpipkt.payload[4] << 8) + ucOpipkt.payload[5];
                            if(opiuce_settsepdn(&comPort,tsedsn%255))
                            {
                                killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                                while(QTime::currentTime() < killTime);
                                if(opiuce_settsepdn(&comPort,tsedsn%255))
                                {
                                    opi_closeuce_com(&comPort);
                                    comPort = 0;
                                    configFailFlg = true;
                                }
                            }
                        }
                        if(opiuce_settserfmode(&comPort, 1)) // default RF mode
                        {
                            killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                            while(QTime::currentTime() < killTime);
                            if(opiuce_settserfmode(&comPort, 1)) // default RF mode
                            {
                                opi_closeuce_com(&comPort);
                                comPort = 0;
                                configFailFlg = true;
                            }
                        }
                        if(opiuce_settserftxpwr(&comPort, 7)) // default RF TX pwr
                        {
                            killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                            while(QTime::currentTime() < killTime);
                            if(opiuce_settserftxpwr(&comPort, 7)) // default RF TX pwr
                            {
                                opi_closeuce_com(&comPort);
                                comPort = 0;
                                configFailFlg = true;
                            }
                        }
                        if(opiuce_settsemmwrite(&comPort, 0)) // default memory module write state
                        {
                            killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                            while(QTime::currentTime() < killTime);
                            if(opiuce_settsemmwrite(&comPort, 0)) // default memory module write state
                            {
                                opi_closeuce_com(&comPort);
                                comPort = 0;
                                configFailFlg = true;
                            }
                        }

                        // do wireless scan to determine best channel to use
                        for(i = 0; i < ZBWLCHANCT; i++)
                        {
                            if(opiuce_setzbchan(&comPort, i+11))
                            {
                                killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                                while(QTime::currentTime() < killTime);
                                if(opiuce_setzbchan(&comPort, i+11))
                                {
                                    opi_closeuce_com(&comPort);
                                    comPort = 0;
                                    configFailFlg = true;
                                }
                            }
                            resWLScan[i] = maxWLMeasure100(&comPort);
                            progQPD.setValue(i);
                            qApp->processEvents();
                        }

                        resMinInd = 0;
                        for(i = 0; i < ZBWLCHANCT; i++)
                        {
                            if(resWLScan[i] >= 0) // measure must have been successful
                            {
                                resMinInd = i;
                                break;
                            }
                        }
                        for(; i < ZBWLCHANCT; i++)
                        {
                            if((resWLScan[i] >= 0) && (resWLScan[i] <= resWLScan[resMinInd]))
                            {
                                resMinInd = i;
                            }
                        }
                        bestzbChan = resMinInd + 11;
                        qDebug() << "best" << bestzbChan;
                        // set zigbee channel
                        if(opiuce_setzbchan(&comPort, bestzbChan))
                        {
                            killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                            while(QTime::currentTime() < killTime);
                            if(opiuce_setzbchan(&comPort, bestzbChan))
                            {
                                opi_closeuce_com(&comPort);
                                comPort = 0;
                                configFailFlg = true;
                            }
                        }
                        if(opiuce_settsezbchan(&comPort, bestzbChan))
                        {
                            killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                            while(QTime::currentTime() < killTime);
                            if(opiuce_settsezbchan(&comPort, bestzbChan))
                            {
                                opi_closeuce_com(&comPort);
                                comPort = 0;
                                configFailFlg = true;
                            }
                        }

                        // remember tse in uce
                        if(opiuce_copytsesettings(&comPort, 0))
                        {
                            killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
                            while(QTime::currentTime() < killTime);
                            if(opiuce_copytsesettings(&comPort, 0))
                            {
                                opi_closeuce_com(&comPort);
                                comPort = 0;
                                configFailFlg = true;
                            }
                        }
                        if(configFailFlg)
                        {
                            signalQGSp->addText("Config failed, close APP and restart.", QFont("Arial", 13))
                                    ->setPos(SIGNALMSGX, SIGNALMSGY);
                                                    }
                        else
                        {
                            signalQGSp->addText("            wear Sensor then Start", QFont("Arial", 13))
                                    ->setPos(SIGNALMSGX, SIGNALMSGY);
                        }
                    }
                    opi_closeuce_com(&comPort);
                    comPort = 0;
                }

                configDoneFlag = true;  // don't do this again
            }

            if(opi_openuce_com(&comPort))
            {
                liveEnableFlg = false;
            }
            else
            {
                if(!ui->startPB->isEnabled())   // if previous time was not on, try to turn tse on
                                                // because don't know if not in or in shutdown mode
                {
                    opiuce_onmode(&comPort);
                    killTime = QTime::currentTime().addMSecs(300);
                    while(QTime::currentTime() < killTime); // wait so that ts has a chance to startup
                    if(!opiuce_getrelaxparams(&comPort, &ucOpipkt))
                    {
                        // figure out what level is based on thm
                        tempui16 = (ucOpipkt.payload[3] << 8) + ucOpipkt.payload[4]; // thm
                        rlevelCalc = (1.7-((float) tempui16)/((float) THM))*10;
                        ui->liveLevelSli->setValue((qint32) (rlevelCalc+0.5));
                    }
                    getRelaxData();  // write data to file if enough data
                    opiuce_resetrelaxdata(&comPort);
                }
                if(opiuce_status(&comPort, &ucOpipkt) || (ucOpipkt.payload[DSNLEN+TSLEN+6+FWVLEN+1] == 0xFF))  // paired before
                {
                    liveEnableFlg = false;
                }
                else
                {
                    liveEnableFlg = true;
                }
            }
            opi_closeuce_com(&comPort); // release device
            comPort = 0;

            if(!liveEnableFlg)
            {
                ui->startPB->setEnabled(false);  // disable appropriate buttons
                ui->liveLevelLa->setEnabled(false);
                ui->liveLevelSli->setEnabled(false);
            }
            else
            {
                ui->startPB->setEnabled(true);  // enable appropriate buttons
                ui->liveLevelLa->setEnabled(true);
                ui->liveLevelSli->setEnabled(true);
            }

        }
        if(ui->startPB->isChecked())    // do live mode stuff
        {
            // Check to see if we need to stop because too long since received a packet
            if(noPktTimerCt++ > MAXTIMERCTMISSPKT)
            {
                ui->startPB->setChecked(false);
                on_startPB_clicked(false);
                return; // get out
            }

            // *** Get Data
            // get new data on comPort and stuff into correct buffers (drawing, fft, led, file, posture viewer)
            getDataRes = opiuce_getwltsedata(&comPort, &tseDataOpipkt);
            if(getDataRes == NEWTSEDATACODE)
            {
                noPktTimerCt = 0;   // reset here
                // get first sample and check sample quality to determine if it should be used
                frmTS = (((qint64) tseDataOpipkt.payload[WFRMHDRLEN-1]) << 40) + (((qint64) tseDataOpipkt.payload[WFRMHDRLEN-1+1]) << 32) +
                        (((qint64) tseDataOpipkt.payload[WFRMHDRLEN-1+2]) << 24) + ((tseDataOpipkt.payload[WFRMHDRLEN-1+3]) << 16) +
                        (tseDataOpipkt.payload[WFRMHDRLEN-1+4] << 8) + (tseDataOpipkt.payload[WFRMHDRLEN-1+5]);
                if(frmTS < prevFrmTS)
                {
                    qDebug() << "error, packets not monotonically increasing";
                }
                prevFrmTS = frmTS;
                adcValue = ((tseDataOpipkt.payload[1+TSLEN+WLFRMHDRLEN]) << 8) + tseDataOpipkt.payload[1+TSLEN+WLFRMHDRLEN+1];
                sampQual = adcValue & SAMPQUALMASK;
                adcValue = adcValue & ~SAMPQUALMASK;

                if(sampQual <= 3)    // take only data with adequate signal quality 0= CRC=1, CRC=0 1=noC; 2=<20C; 3=>20C;
                {
                    accxQV.resize(1);
                    accyQV.resize(1);
                    acczQV.resize(4);

                    // if we took it, then hurry and get UCE calculated FFT data
                    if(opiuce_getwltsefft(&comPort, &tseFFTOpipkt))
                    {
                        qDebug() << "error getting fft";
                        return;
                    }
                    tsQV.append(frmTS);
                    skpQV.append(tseDataOpipkt.payload[WFRMHDRLEN+TSLEN] >> 7);
                    batQV.append(tseDataOpipkt.payload[WFRMHDRLEN+TSLEN+1] & 0x01);
                    // handle signal data
                    signalDataToDrawQQ.enqueue(adcValue);
                    signalDataToFFTQQ.enqueue(adcValue);
                    adcQV.append(adcValue);
                    if(tseDataOpipkt.length == TSEFRMLEN) // full 64 adc samples here
                    {
                        for(i = 1; i < ADCLEN; i++) // already got first sample
                        {
                            adcValue = ((tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*i]) << 8) + tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*i+1];
                            signalDataToDrawQQ.enqueue(adcValue);
                            signalDataToFFTQQ.enqueue(adcValue);
                            adcQV.append(adcValue);
                        }
                        tmpQV.append(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*ADCLEN] << 4);
                        axQV.append(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*ADCLEN+TMPLEN] << 8);
                        accxQV[0]=(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*ADCLEN+TMPLEN] << 8);
                        ayQV.append(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*ADCLEN+TMPLEN+1] << 8);
                        accyQV[0]=(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*ADCLEN+TMPLEN+1] << 8);
                        for(k = 0; k < ACCLEN; k++)
                        {
                            azQV.append(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*ADCLEN+TMPLEN+2+k] << 8);
                            acczQV[k]=(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*ADCLEN+TMPLEN+2+k] << 8);
                        }
                        edQV.append(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*ADCLEN+TMPLEN+ACCDLEN]);
                    }
                    else if(tseDataOpipkt.length == (TSEFRMLEN-4)) // only 62 adc samples
                    {
                        for(i = 1; i < ADCLEN-2; i++) // already got first sample
                        {
                            adcValue = ((tseDataOpipkt.payload[1+TSLEN+WLFRMHDRLEN+2*i]) << 8) + tseDataOpipkt.payload[1+TSLEN+WLFRMHDRLEN+2*i+1];
                            signalDataToDrawQQ.enqueue(adcValue);
                            signalDataToFFTQQ.enqueue(adcValue);
                            adcQV.append(adcValue);
                        }                        
                        tmpQV.append(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*(ADCLEN-2)] << 4);
                        axQV.append(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*(ADCLEN-2)+TMPLEN] << 8);
                        accxQV[0]=(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*(ADCLEN-2)+TMPLEN] << 8);
                        ayQV.append(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*(ADCLEN-2)+TMPLEN+1] << 8);
                        accyQV[0]=(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*(ADCLEN-2)+TMPLEN+1] << 8);
                        for(k = 0; k < ACCLEN; k++)
                        {
                            azQV.append(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*(ADCLEN-2)+TMPLEN+2+k] << 8);
                            acczQV[k]=(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*(ADCLEN-2)+TMPLEN+2+k] << 8);
                        }
                        edQV.append(tseDataOpipkt.payload[WFRMHDRLEN-1+TSLEN+WLFRMHDRLEN+2*(ADCLEN-2)+TMPLEN+ACCDLEN]);
                    }
                    if(sampQual>0)
                    {
                        sqQV.append(-1000*sampQual); //*-1.5dB
                        qDebug() << sampQual;
                    }
                    else
                        sqQV.append(calcAct(&axQV, &ayQV, &azQV, &newstep, &azold, &axold, &ayold, &az4old, &ax2old, &ay2old, &ax3old, &ay3old, &ax4old, &ay4old, &az8old, &az12old, &az16old));

                    // handle fft data
                    for(i = 0; i < UCEFFTLEN; i++)
                    {
                        uceFFTDataToProcessQV.append((tseFFTOpipkt.payload[1+2*i]<< 8) + tseFFTOpipkt.payload[1+2*i+1]);
                    }
                }
            }

            // *** Data processing
            // write to file if necessary
            // check if enough data to write into EDF, adc will always have the least
            if(adcQV.size() > ADCLEN*FRMSPERSEC*EDFDRDURSEC)
            {
                // process vectors to add missing frames
                procQV(&tsQV, &skpQV, &batQV, &adcQV,
                       &tmpQV, &axQV, &ayQV, &azQV, &sqQV, &edQV);

                // add data at beginning (first frame) to fill up to floor(second)
                if(!wroteDRCt)
                {
                    stQDT = QDateTime::currentDateTime();   // system time
                    outQFp = new QFile(QString("E%1_%2.edf").arg(stQDT.toString("yyyyMMdd_hhmmss")).arg(tsepdn));
                    this->setWindowTitle(QString("ReLax : %1").arg(outQFp->fileName()));
                    if(outQFp->exists())
                    {
                        msgQMB.setWindowTitle("Out File Exists");
                        msgQMB.setText("Out file already exists in selected directory.");
                        msgQMB.setInformativeText("Do you want to overwrite?");
                        msgQMB.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
                        if(msgQMB.exec() == QMessageBox::Cancel)
                        {
                            delete outQFp;
                            outQFp = NULL;
                            return;
                        }
                    }
                    if(!outQFp->open(QIODevice::WriteOnly))
                    {
                        delete outQFp;
                        outQFp = NULL;
                        return;
                    }
                    outQDSp = new QDataStream(outQFp);
                    outQDSp->setByteOrder(QDataStream::LittleEndian);
                    // write EDF header
                    localPatientID = QString("X X X X");

                    localRecordID = QString("X X %1_OPITSE%2").arg(localUTCOffset()).arg(tsepdn,3,10,QChar('0'));
                    localRecordID.prepend(stQDT.toString("-yyyy "));
                    currMonth = stQDT.date().month();      // have to do this because of QDateTime letters being 2 chars
                    if(currMonth == 1)  localRecordID.prepend("JAN");
                    else if(currMonth == 2) localRecordID.prepend("FEB");
                    else if(currMonth == 3) localRecordID.prepend("MAR");
                    else if(currMonth == 4) localRecordID.prepend("APR");
                    else if(currMonth == 5) localRecordID.prepend("MAY");
                    else if(currMonth == 6) localRecordID.prepend("JUN");
                    else if(currMonth == 7) localRecordID.prepend("JUL");
                    else if(currMonth == 8) localRecordID.prepend("AUG");
                    else if(currMonth == 9) localRecordID.prepend("SEP");
                    else if(currMonth == 10) localRecordID.prepend("OCT");
                    else if(currMonth == 11) localRecordID.prepend("NOV");
                    else localRecordID.prepend("DEC");
                    localRecordID.prepend(stQDT.toString("dd-")).prepend("Startdate ");

                    // write EDF header, will need to come back later to adjust number of data records
                    // which are unknown (-1) at this point
                    // also pass the dataType and use the write header if need to write events
                    edfEhdropiwrite(outQDSp, &localPatientID, &localRecordID, &stQDT, -1);

                    firstFrmTS = tsQV.first();
                    tagTSQV.prepend(firstFrmTS);
                    tagTextQV.prepend("DataStart");
                    // difference between floored exact second and first time in frames
                    //beginOffFrms = ((double) (stQDT.toMSecsSinceEpoch() % 1000))*512.0/1000/ADCLEN;
                    beginOffFrms = ((double) (stQDT.toMSecsSinceEpoch() % 1000))*TSERTCFREQ/1000/ADCLEN;
                    beginOffFrmsi = (qint32) (beginOffFrms);
                    beginOffFrmsj = beginOffFrms-((double) beginOffFrmsi);
                    // put in default values for beginning in constant frame units
                    for(; beginOffFrmsi > 0; beginOffFrmsi--)
                    {
                        //tsQV.prepend(mmFirstFrmTS-512*beginOffFrmsi);
                        tsQV.prepend(tsQV.first()-ADCLEN/TSERTCFREQ*UCERTCFREQ*beginOffFrmsi);
                        skpQV.prepend(0);
                        batQV.prepend(1);
                        sqQV.prepend(0);
                        edQV.prepend(0);
                        for(k = 0; k < ADCLEN; k++)
                            adcQV.prepend(0);
                        for(k = 0; k < TMPLEN; k++)
                            tmpQV.prepend(1024);   // temp ~25
                        for(k = 0; k < ACCLEN/4; k++)
                        {
                            axQV.prepend(0);
                            ayQV.prepend(0);
                        }
                        for(k = 0; k < ACCLEN; k++)
                            azQV.prepend(0);
                    }

                    // add less than a frame data
                    if(beginOffFrmsj > 0.5)
                    {
                        tsQV.prepend(tsQV.first()-ADCLEN/TSERTCFREQ*UCERTCFREQ*(beginOffFrmsi+1));
                        skpQV.prepend(0);
                        batQV.prepend(1);
                        sqQV.prepend(0);
                        edQV.prepend(0);
                        tmpQV.prepend(1024);   // temp ~25
                        axQV.prepend(0);
                        ayQV.prepend(0);
                    }
                    for(k = 0; k < ((qint32) (beginOffFrmsj*ADCLEN)); k++)
                        adcQV.prepend(0);
                    for(k = 0; k < ((qint32) (beginOffFrmsj*ACCLEN)); k++)
                        azQV.prepend(0);

                    // fix up tag times by adding
                    for(k = 0; k < tagTSQV.size(); k++)
                        tagTSQV.replace(k, tagTSQV.at(k)+((qint64) (beginOffFrms*ADCLEN/TSERTCFREQ*UCERTCFREQ)));
                }

                // write to file, add any tags that are within the timestamp ranges
                if(outQDSp != 0)
                {
                    dataRecCt = edfEwrite(outQDSp, &adcQV, &tmpQV, &axQV, &ayQV, &azQV, &sqQV, wroteDRCt, firstFrmTS, &tsQV, &tagTSQV, &tagTextQV);
                    wroteDRCt += dataRecCt;

                    // clip off the parts it wrote
                    tsQV.remove(0, dataRecCt*1*FRMSPERSEC*EDFDRDURSEC);
                    skpQV.remove(0, dataRecCt*1*FRMSPERSEC*EDFDRDURSEC);
                    batQV.remove(0, dataRecCt*1*FRMSPERSEC*EDFDRDURSEC);
                    adcQV.remove(0, dataRecCt*ADCLEN*FRMSPERSEC*EDFDRDURSEC);
                    tmpQV.remove(0, dataRecCt*TMPLEN*FRMSPERSEC*EDFDRDURSEC);
                    axQV.remove(0, dataRecCt*ACCLEN/4*FRMSPERSEC*EDFDRDURSEC);
                    ayQV.remove(0, dataRecCt*ACCLEN/4*FRMSPERSEC*EDFDRDURSEC);
                    azQV.remove(0, dataRecCt*ACCLEN*FRMSPERSEC*EDFDRDURSEC);
                    sqQV.remove(0, dataRecCt*1*FRMSPERSEC*EDFDRDURSEC);
                    edQV.remove(0, dataRecCt*1*FRMSPERSEC*EDFDRDURSEC);
                }
            }

            // Do BIG FFT if have enough points and have already drawn previous FFT
            if((signalDataToFFTQQ.size() >= LIVEFFTLEN) && (spectDataToDrawQV.size() == 0))
            {
                for(i = 0; i < LIVEFFTLEN; i++)
                {
                    timeData[i] = (float) signalDataToFFTQQ.dequeue();
                }

                myQFT.forwardTransform(timeData,freqData);
                freqQCFQV = myQFT.toComplex(freqData);
                for(i = 0; i < LIVEFFTLEN/2; i++)
                {
                    tempDoub = (double) (freqQCFQV[i].real()*freqQCFQV[i].real() + freqQCFQV[i].imaginary()*freqQCFQV[i].imaginary());
                    if(tempDoub <= 0)  // can't log 0 or negative numbers
                        tempDoub = 1;
                    else
                        tempDoub = LIVEFFTDBGAIN*(10*log10(tempDoub*(i+1))-70- LIVEFFTDBOFF);   // with freq. equalization
                    spectDataToDrawQV.append((float) tempDoub);
                }

                // put back data according to overlap, so can be used for next fft
                // must do backwards to make sure data is in right order
                for(i = LIVEFFTLEN-1; i >= (LIVEFFTLEN-LIVEFFTLENOVERLAP); i--)
                {
                    signalDataToFFTQQ.prepend((qint16) timeData[i]);
                }
            }

            // if got SMALL fft data from uce then process
            if(uceFFTDataToProcessQV.size() >= UCEFFTLEN)
            {
                // calculations (NO averaging: M dynamic range too wide)
                lCalc = (uceFFTDataToProcessQV[1] + uceFFTDataToProcessQV[2] +  //1x
                         uceFFTDataToProcessQV[3]);
                bCalc = (uceFFTDataToProcessQV[4] + uceFFTDataToProcessQV[5] +  //1x
                         uceFFTDataToProcessQV[6] + uceFFTDataToProcessQV[7] +
                         uceFFTDataToProcessQV[8] + uceFFTDataToProcessQV[9] + uceFFTDataToProcessQV[10]);
                gCalc = (uceFFTDataToProcessQV[17] + uceFFTDataToProcessQV[18] +  // 1x
                         uceFFTDataToProcessQV[19] + uceFFTDataToProcessQV[20] +
                         uceFFTDataToProcessQV[21] + uceFFTDataToProcessQV[22] +
                         uceFFTDataToProcessQV[23] + uceFFTDataToProcessQV[24]);
                mCalc = (uceFFTDataToProcessQV[25] + uceFFTDataToProcessQV[26] +  //1x
                         uceFFTDataToProcessQV[27] + uceFFTDataToProcessQV[28] +
                         uceFFTDataToProcessQV[29] + uceFFTDataToProcessQV[30] + uceFFTDataToProcessQV[31]);

                if(gCalc>mCalc) gmCalc = (gCalc) - mCalc;
                else gmCalc = 0;
                if(bCalc>mCalc*2) bmCalc = (bCalc) - mCalc*2;
                else bmCalc = 0;

                if(mCalc >= thx) stRelax_now = LEDNON;      //exclude if HF spike absolute
                else if((lCalc > offl) && (lCalc > mCalc)) stRelax_now = LEDNON; //exclude if LF spike absolute
                else if(mCalc >= thm || (mCalc > (thm/2) && gmCalc<=0)) //absolute or relative
                {
                    stRelax_now = LEDRED;
                }
                else if((gmCalc > th3gm) || gCalc > (offm+th3gm) ) //absolute or relative
                {
                    stRelax_now = LEDRED;
                }
                else if((gmCalc > th2gm) || gCalc > (offm+th2gm) || (bmCalc > th2bm) || bCalc > (offm*2+th2bm)) //absolute or relative
                {
                    stRelax_now = LEDORG;
                }
                else if((gmCalc > th1gm) || gCalc > (offm+th1gm) || (bmCalc > th1bm) || bCalc > (offm*2+th1bm)) //absolute or relative
                {
                    stRelax_now = LEDGRN;
                }
                else
                {
                    stRelax_now = LEDBLU;
                }
                {
                    ledDataToDrawQQ.enqueue(stRelax_now);
                    stRelax_prev = stRelax_now;
                }
                if(stRelax_now) // only add if nonzero
                {
                    stRelax_accum += stRelax_now;
                    stRelax_pktCt++;
                }
                uceFFTDataToProcessQV.clear(); // delete data since has been used
            }

            // *** Graphics
            //update signal stuff
            //data used automatically removed from queue
            if(signalDataToDrawQQ.size() >= MINSIGDATAPTSTODRAW)
            {
                qp = new QPainterPath();
                for(i = 0; i < MINSIGDATAPTSTODRAW;i++)
                {

                    signalDrawCount++;
                    //clear screen
                    if(signalDrawCount*signalincrease > ui->signalGV->width())
                    {
                        signalDrawCount=0;
                        if(ui->GraphicsDrawCheckBox->isChecked())
                        signalQGSp->addPath((*qp),QPen(Qt::red));
                        delete qp;
                        qp = new QPainterPath();
                        signalQGSp->clear();
                    }

                    if(firstTimeDrawsignal)
                    {
                        nowvalue=signalDataToDrawQQ.dequeue()*signalZoomRatio;
                        beforevalue= nowvalue;
                        firstTimeDrawsignal = false;
                    }
                    else
                    {
                        beforevalue = nowvalue;
                        nowvalue=signalDataToDrawQQ.dequeue()*signalZoomRatio;
                    }
                    //clamp
                    if(nowvalue>MAX_MIN_ADC)
                        nowvalue=MAX_MIN_ADC;
                    else if(nowvalue<-1*MAX_MIN_ADC)
                        nowvalue=-1*MAX_MIN_ADC;
                    if(beforevalue>MAX_MIN_ADC)
                        beforevalue=MAX_MIN_ADC;
                    else if(beforevalue<-1*MAX_MIN_ADC)
                        beforevalue=-1*MAX_MIN_ADC;
                    qp->moveTo(signalbegin+(signalDrawCount-1)*signalincrease,screenmiddle-(beforevalue-signalmiddle)*scalerate);
                    qp->lineTo(signalbegin+signalDrawCount*signalincrease,screenmiddle-(nowvalue-signalmiddle)*scalerate);
                }
                if(ui->GraphicsDrawCheckBox->isChecked())
                signalQGSp->addPath((*qp),QPen(Qt::red));
                delete qp;
            }
            if(spectDataToDrawQV.size() == LIVEFFTLEN/2) //BIG FFT
            {
                if((spectDrawCount >= 0) && (spectDrawCount < (spectQGSp->width()/FFTPENWIDTH)))
                    spectDrawCount++;   // update to next place to draw
                else
                {
                    ledQGSp->clear();
                    spectQGSp->clear(); // clear screen
                    spectDrawCount=0;   // start over
                    ledDrawCount = 0;   // make sure led and spectrogram are always aligned
                }
                //draw spectrogram
                //to save the parameter for fft
                QColor ffthsl;
                QPen   fftPen;
                float fft_zoomrate =1;
                //double rr,gg,bb;
                double fftnowvalue = 0;
                float spectincrease;  //decide the gap between every points(x-axis)
                float increase_log[NUMPATHFFT/2+1]; //decide the gap between every points(x-axis)in LOG scale & as FFT_equalization
                int j;
                int tempy;

                for( i=0; i <= LIVEFFTLEN/2; i++)
                {
                    increase_log[i]=10*log10((i+1));//10db/dec equalization && draw segment length
                }
                tempy=button;
                //decide the gap between every points(y-axis)
                if(numScalefft>LIVEFFTLEN/2)  //display to max of 256Hz
                    numScalefft=LIVEFFTLEN/2;
                // add LOG scale increase here
                spectincrease=(float)ui->spectGV->height();
                spectincrease=spectincrease/((float)(increase_log[numScalefft]-increase_log[1]));  //now normalized to display section
                if((!ui->spectEnablePB->isChecked()) && (ui->GraphicsDrawCheckBox->isChecked())) // only draw if necessary
                {
                    for(j=0 ;j < numScalefft; j++)
                    {
                        //distribute the data
                        //fftnowvalue= j; //effected_by_ko_co_fft[j];
                        fftnowvalue = spectDataToDrawQV.at(j);
                        fftnowvalue=fftnowvalue*fft_zoomrate;
                        //clamp
                        if (fftnowvalue>FFTSIGNALMAX) fftnowvalue=FFTSIGNALMAX;
                        if (fftnowvalue<FFTSIGNALMIN)  fftnowvalue=FFTSIGNALMIN;
                        //convert to pseudo-color
                        ffthsl = calcSpectRectColor(fftnowvalue);
                        fftPen.setColor(ffthsl);
                        spectQGSp->addRect(spectbegin+(spectDrawCount-1)*FFTPENWIDTH,
                                           button-(increase_log[j]-increase_log[1])*spectincrease,
                                           FFTPENWIDTH,
                                           tempy-(button-(increase_log[j]-increase_log[1])*spectincrease),
                                           fftPen,QBrush(ffthsl));
                        tempy=button-(increase_log[j]-increase_log[1])*spectincrease;
                    }//for(int j=2;j<=numScalefft;j++) end
                }
                spectDataToDrawQV.clear();  // delete data that was drawn
            }
            if(!ledDataToDrawQQ.isEmpty()) //small FFT
            {
                QPen ledPen;
                QBrush ledBrush;

                if((ledDrawCount >= 0) && (ledDrawCount < (ledQGSp->width()/LEDPENWIDTH)))
                    ledDrawCount++;
                else
                {
                    ledQGSp->clear();
                    spectQGSp->clear(); // clear screen
                    ledDrawCount = 0;
                    spectDrawCount=0;   // make sure led and spectrogram are always aligned
                }
                quint8 currSt = ledDataToDrawQQ.dequeue();
                if(currSt == LEDRED)
                {
                    ledPen = QPen(Qt::red);
                    ledBrush = QBrush(Qt::red);
                }
                else if(currSt == LEDORG)
                {
                    ledPen = QPen(Qt::yellow);    // find a better color
                    ledBrush = QBrush(Qt::yellow);
                }
                else if(currSt == LEDGRN)
                {
                    ledPen = QPen(Qt::green);
                    ledBrush = QBrush(Qt::green);
                }
                else if(currSt == LEDBLU)
                {
                    ledPen = QPen(Qt::blue);
                    ledBrush = QBrush(Qt::blue);
                }
                else
                {
                    ledPen = QPen(Qt::black);  // NON
                    ledBrush = QBrush(Qt::black);
                }
                if(ui->GraphicsDrawCheckBox->isChecked())
                ledQGSp->addRect(ui->ledGV->x()+(ledDrawCount-1)*LEDPENWIDTH,
                                 ui->ledGV->y(), LEDPENWIDTH, ui->ledGV->height(),ledPen, ledBrush);
                // display the score in live mode only if got data, also check if zero since divide
                if(stRelax_pktCt) ui->RLScoreLa->setText(QString("RL%1.%2").arg(stRelax_accum/stRelax_pktCt).arg((stRelax_accum*100)/stRelax_pktCt-(stRelax_accum/stRelax_pktCt)*100));
//                qDebug() << "LED data" << currSt << lCalc << bCalc << gCalc << mCalc <<"**"<< gmCalc << bmCalc;
            }
            if(acczQV.size() == 4) // only if we actually got data
            {
                if(ui->GraphicsDrawCheckBox->isChecked())
                TDVp->livedisplayroutine(&accxQV,&accyQV,&acczQV);
                accxQV.clear();
                accyQV.clear();
                acczQV.clear();
            }
        }
    }
    else // if it is not our timer, then pass the event to super class timerEvent
    {
        QWidget::timerEvent(event);
    }
    timerEventCt++;
}


// Convert to pseudo-color
QColor ReLax::calcSpectRectColor(double value)
{
    double gg, rr, bb;
    QColor retVal;

    gg=(qint32) value;
    if(gg<32)
    {
        rr=255;
        bb=32+gg*3;
    }
    else if(gg<224)
    {
        rr=298-gg*4/3;
        bb=128;
    }
    else
    {
        rr=0;
        bb=gg*4-765;
    }
    retVal.setHsl(rr,255,bb);
    return retVal;
}


void ReLax::on_modeSli_valueChanged(int value)
{
    signalQGSp->clear();
    spectQGSp->clear();
    ledQGSp->clear();

    if(value == LIVEMODE)
    {
        // limit screen size to current
        this->setMinimumSize(this->width(),435);
        this->setMaximumSize(this->width(),435);
        ui->GraphicsDrawCheckBox->show();
        if(!timer.isActive())  // make sure timer is on
        {
            timer.start(UPDATEPERIOD, this);
        }
        signalQGSp->addText("            wear Sensor then Start", QFont("Arial", 13))
                ->setPos(SIGNALMSGX, SIGNALMSGY);
    }
    else
    {
        // limit screen size to current
        this->setMinimumSize(this->width(),450);
        this->setMaximumSize(this->width(),450);
        ui->GraphicsDrawCheckBox->hide();
        if(timer.isActive())    // make sure timer is off
        {
            timer.stop();
        }
        ui->startPB->setEnabled(true); // turn on startPB
    }

}


void ReLax::drawSavedSpect()
{
    long countfft = 0, countadc=0 ;
    double ffttemp;
    int i=0,j=0;
    float skipfft=1;
    long counteffected = 0;
    //to save the data for drawing
    float saveFFT[NUMPATHFFT];//to save raw data,uncaculated to fft,just adc
    float calculatedFFT[NUMPATHFFT]; //to save the data after FFT calculation
    int numpathfftcol = 500;
    int countfftcolumn = 0;
    double re = 0, im = 0;
    double fftk=FFTKO;  // set initial gain multiplier 8=32db 6=43db dynamic range
    double fftco=FFTCO;   // (-)fftco value 1 => -70-1=-71 db for 1024FFT (-70db built-in)
    int numScalefft=NUMPATHFFT/2; //256 bins, 128Hz BW
    QVector<QComplexFloat> spectDataToDrawQV; // to save the result fft data uneffected by ko co
    QFourierTransformer transformer;  //should Setting a fixed size for the transformation
    transformer.setSize(NUMPATHFFT);
    countadc = index_begin + show_num_of_seconds*SAMPLERATEADC;
    //countadc = signalDataQvp.size();
    skipfft=(float (show_num_of_seconds*SAMPLERATEADC*2))/NUMPATHFFT; //avoid truncation
    skipfft=(skipfft-1)/(numpathfftcol); //skip factor
    if(skipfft<=1) skipfft=1;
    //fft
    //from here
    i=index_begin;
    while (i<countadc)  //for full FFT
    {
        if(i>=signalDataQvp.size())
            break;
        if(countfft>=0&&countfft<NUMPATHFFT){
            saveFFT[countfft]=signalDataQvp.at(i); //firsttime shift
            countfft++;
            i++;
        }
        else
        {
            //full sample
            if(countfftcolumn>=0&&countfftcolumn<numpathfftcol)
            {
                countfftcolumn++;
                for(j=0;j<NUMPATHFFT;j++) //add Hanning window
                    saveFFT[j]=0.5*(1-qCos((2*M_PI*j)/(NUMPATHFFT-1)))*saveFFT[j]; //add Hanning window
                transformer.forwardTransform(saveFFT,calculatedFFT);
                spectDataToDrawQV=transformer.toComplex(calculatedFFT);
                //effected by fftk and fftco
                for(j=0;j<NUMPATHFFT/2;j++)
                {
                    re=(double)spectDataToDrawQV[j].real()*(double)spectDataToDrawQV[j].real();
                    im=(double)spectDataToDrawQV[j].imaginary()*(double)spectDataToDrawQV[j].imaginary();
                    ffttemp=re+im;
                    if(ffttemp<=0)  //can't log10(0)
                        qqscenep->effected_by_ko_co_fft[counteffected][j]= 0;//in db
                    else
                        qqscenep->effected_by_ko_co_fft[counteffected][j]=fftk*((10*log10(ffttemp*(j+1)))-70-fftco);//in db + equalization
                }//for(int j=0;j<NUMPATHFFTROW/2;J++) end
                counteffected++;
                if(skipfft>1) //with FFT skip
                {
                    countfft=0;
                    i += float ((skipfft-2.0) * (NUMPATHFFT/2)); //skip forward, avoid truncation
                    if(i>=countadc) break;
                }
                else // no skip
                {
                    countfft=NUMPATHFFT/2; //halfway
                    for(j=0;j<countfft;j++)
                        saveFFT[j]=saveFFT[countfft+j];
                }
                saveFFT[countfft]=signalDataQvp.at(i); //1st new data point
                countfft++;
                i++;
            }
            else{ //screen full
                break;
            }//else end
        }//if(countfft>=0&&countfft<NUMPATHFFT) end
    }//while end
    qqscenep->setAttribute(Qt::WA_OpaquePaintEvent,false);
    qqscenep->shifteindex = 0;
    qqscenep->setData(numScalefft,counteffected);
    qqscenep->todraw = true;
    qqscenep->countdraw = 0;
    qqscenep->repaint();
}


void ReLax::drawSavedSignal()
{
    //parameter for drawing
    float   begin;
    float everysecondsignalincrease,everywidthsignalincrease;
    float scalerate;
    float nowvaluey=0,beforevaluey,nowvaluex=0,beforevaluex;
    float nextvaluey;
    float signalmiddle;
    bool  seperatestart,notenough;
    int i, reduction,j; //reduction factor to display fewer data
    float samplerate = SAMPLERATEADC;

    signalQGSp->clear();
    QPainterPath qp;
    //decide where to begin to draw
    begin=ui->signalGV->x();
    //decide the gap between every points(x-axis)
    everywidthsignalincrease=((float)ui->signalGV->width())/(float)show_num_of_seconds;
    everysecondsignalincrease=everywidthsignalincrease/(float)samplerate;
    if(everysecondsignalincrease<0.25) reduction=(0.25/everysecondsignalincrease); //max 4 values per pixel
    else reduction=1;
    if(samplerate>=256 && reduction>256) reduction=256;
    else if(samplerate>=256 && reduction>128) reduction=128;
    else if(samplerate>=256 && reduction>64) reduction=64;
    else if(samplerate>=256 && reduction>32) reduction=32;
    else if(samplerate>=16 && reduction>16) reduction=16;
    else if(samplerate>=16 && reduction>8) reduction=8;
    else if(samplerate>=4 && reduction>4) reduction=4;
    else if(samplerate>=4 && reduction>2) reduction=2;
    else if(samplerate>=4 && reduction>1) reduction=1;
    else reduction=1;
    everysecondsignalincrease=everysecondsignalincrease*reduction; //adjust by reduction factor

    //decide the scale of the signal(y-axis),scalerate=(scene'height)/(signal range you want to show)
    scalerate=((float)(ui->signalGV->height()))/(float)(2*ADCDIGITALMAX); //ratio to max-min,-1 to 1
    //decide the middle value of signal
    signalmiddle=0;

    if(index_begin<signalDataQvp.count())
        beforevaluey=(float)signalDataQvp.at(index_begin)*signalZoomRatio;
    else
        return ;
    beforevaluey=(changablemiddle)-(beforevaluey-signalmiddle)*scalerate;
    beforevaluex=begin;
    notenough=false;


    for(i=0;i<show_num_of_seconds;i++)//how many seconds
    {
        if(index_begin+samplerate*i<signalDataQvp.count())
            nowvaluey=(float)signalDataQvp.at(index_begin+samplerate*i)*signalZoomRatio;
        else
            break;
        nowvaluey=(changablemiddle)-(nowvaluey-signalmiddle)*scalerate;
        if(index_begin+samplerate*i+1<signalDataQvp.count())
            nextvaluey=(float)signalDataQvp.at(index_begin+samplerate*i+1)*signalZoomRatio;
        else
            break;
        nextvaluey=(changablemiddle)-(nextvaluey-signalmiddle)*scalerate;

        nowvaluex=begin;
        seperatestart=false;
        for(j=0;j<samplerate/reduction;j++) //how many sample per second adj. for reduction
        {
            if(seperatestart)
            {
                if(index_begin+samplerate*i+j*reduction<signalDataQvp.count()) //reduction
                    nowvaluey=(float)signalDataQvp.at(index_begin+samplerate*i+j*reduction)*signalZoomRatio; //reduction
                else
                    goto FINALDRAW;
                nowvaluey=(changablemiddle)-(nowvaluey-signalmiddle)*scalerate;
                if(index_begin+samplerate*i+j*reduction+1<signalDataQvp.count()) //reduction
                    nextvaluey=(float)signalDataQvp.at(index_begin+samplerate*i+j*reduction+1)*signalZoomRatio; //reduction
                nextvaluey=(changablemiddle)-(nextvaluey-signalmiddle)*scalerate;
                nowvaluex=nowvaluex+everysecondsignalincrease; //reduction
            }//if end
            seperatestart=true;
            qp.moveTo(beforevaluex,beforevaluey);
            qp.lineTo(nowvaluex,nowvaluey);
            beforevaluey=nowvaluey;
            beforevaluex=nowvaluex;
        }//samples end for end
        begin=ui->signalGV->x()+everywidthsignalincrease*(i+1);
    }//seconds end
FINALDRAW:
    if(index_begin+samplerate*show_num_of_seconds<signalDataQvp.count())
    {
        nowvaluey=(float)signalDataQvp.at(index_begin+samplerate*show_num_of_seconds)*signalZoomRatio;
        nowvaluey=(changablemiddle)-(nowvaluey-signalmiddle)*scalerate;
        qp.moveTo(beforevaluex,beforevaluey);
        qp.lineTo(nowvaluex,nowvaluey);
    }
    else
    {
        qp.moveTo(beforevaluex,beforevaluey);
        qp.lineTo(nowvaluex,nowvaluey);
    }
    signalQGSp->addPath(qp,QPen(Qt::red));
}


bool ReLax::convertedf()
{
    QFile *infilep;
    QDataStream *instrp;
    quint8 pdnListp[PDNLISTLEN];
    QVector<qint64> *tsQVpp[PDNLISTLEN];
    QVector<quint8> *skpQVpp[PDNLISTLEN];
    QVector<quint8> *batQVpp[PDNLISTLEN];
    QVector<qint16> *adcQVpp[PDNLISTLEN];
    QVector<qint16> *axQVpp[PDNLISTLEN];
    QVector<qint16> *ayQVpp[PDNLISTLEN];
    QVector<qint16> *azQVpp[PDNLISTLEN];
    QVector<qint16> *tmpQVpp[PDNLISTLEN];
    QVector<qint16> *sqQVpp[PDNLISTLEN];
    QVector<quint8> *edQVpp[PDNLISTLEN];
    QVector<qint64> annOnsetTSQV;
    QVector<QString> annTextQV;

    QString  lpid, lrid;
    QDateTime startDT;
    qint32 numDataRecs, dataRecDur;
    qint32 numSignals;
    QVector<QString> labelSignalsQV, transTypeQV, physDimQV, prefiltQV;
    QVector<qint32> physMinQV, physMaxQV, digMinQV, digMaxQV;
    QVector<qint32> sampsPerDRQV;
    qint32 pdnSlot, i;
    qint32 pdnDes;
    QMessageBox msgBox;
    signalDataQvp.clear();
    //set message window title
    msgBox.setWindowTitle("Warnning");
    msgBox.setStyleSheet("background-image: url(:/images/bioshare-BGtiletexturedMixedGloss600px.jpg);");

    // Opening of inputs/outputs and Error Checking
    infilep = new QFile(ui->savedFilenameLE->text());
    if (!infilep->open(QIODevice::ReadOnly))
    {
        msgBox.setText(QString("Problem with input file \"%1\"").arg(ui->savedFilenameLE->text()));
        msgBox.exec();
        delete infilep;
        return false;
    }

    // open input file stream
    instrp = new QDataStream(infilep);
    instrp->setByteOrder(QDataStream::LittleEndian);

    // get header information
    edfhdrread(instrp, &lpid, &lrid, &startDT, &numDataRecs, &dataRecDur,
               &numSignals, &labelSignalsQV, &transTypeQV, &physDimQV, &physMinQV,
               &physMaxQV, &digMinQV, &digMaxQV, &prefiltQV, &sampsPerDRQV);

    // init qvector pointers to null
    for(pdnSlot = 0; pdnSlot < PDNLISTLEN; pdnSlot++)
    {
        tsQVpp[pdnSlot] = 0;
        skpQVpp[pdnSlot] = 0;
        batQVpp[pdnSlot] = 0;
        adcQVpp[pdnSlot] = 0;
        tmpQVpp[pdnSlot] = 0;
        axQVpp[pdnSlot] = 0;
        ayQVpp[pdnSlot] = 0;
        azQVpp[pdnSlot] = 0;
        sqQVpp[pdnSlot] = 0;
        edQVpp[pdnSlot] = 0;
    }

    // get PDN from Device field
    pdnDes = getPDNlrid(lrid);
    pdnSlot = 0;

    // copy pdn's
    for(i = 0; i < PDNLISTLEN; i++)
        pdnListp[i] = 0xFF;
    pdnListp[pdnSlot] = pdnDes;

    // initialize qvectors
    tsQVpp[pdnSlot] = new QVector<qint64>(0);
    skpQVpp[pdnSlot] = new QVector<quint8>(0);
    batQVpp[pdnSlot] = new QVector<quint8>(0);
    adcQVpp[pdnSlot] = new QVector<qint16>(0);
    tmpQVpp[pdnSlot] = new QVector<qint16>(0);
    axQVpp[pdnSlot] = new QVector<qint16>(0);
    ayQVpp[pdnSlot] = new QVector<qint16>(0);
    azQVpp[pdnSlot] = new QVector<qint16>(0);
    sqQVpp[pdnSlot] = new QVector<qint16>(0);
    edQVpp[pdnSlot] = new QVector<quint8>(0);

    QVector <qint16> accxqvect;
    QVector <qint16> accyqvect;
    QVector <qint16> acczqvect;
    QVector <qint64> mytsQV;

   // if(edfDread(instrp, startDT, numDataRecs, dataRecDur, numSignals,
   //             sampsPerDRQV, tsQVpp[pdnSlot], skpQVpp[pdnSlot], batQVpp[pdnSlot],
   //             &signalDataQvp, tmpQVpp[pdnSlot], axQVpp[pdnSlot], ayQVpp[pdnSlot],
   //             azQVpp[pdnSlot], sqQVpp[pdnSlot], edQVpp[pdnSlot],
    //            &annOnsetTSQV, &annTextQV) < 0)
    if(edfDread(instrp, startDT, numDataRecs, dataRecDur, numSignals,
                    sampsPerDRQV, &mytsQV, skpQVpp[pdnSlot], batQVpp[pdnSlot],
                    &signalDataQvp, tmpQVpp[pdnSlot], &accxqvect, &accyqvect,
                    &acczqvect, sqQVpp[pdnSlot], edQVpp[pdnSlot],
                    &annOnsetTSQV, &annTextQV) < 0)
    {
        delspecQVs(pdnSlot, tsQVpp, skpQVpp, batQVpp, adcQVpp, tmpQVpp, axQVpp, ayQVpp, azQVpp, sqQVpp, edQVpp);
    }

    if(TDVp != NULL)
    {
        delete TDVp;
        TDVp = NULL;
    }
    TDVp = new twoDaccelviewer(false,ui->savedFilenameLE->text().remove(0,ui->savedFilenameLE->text().lastIndexOf("/")+1));
    TDVp ->setData(&accxqvect,&accyqvect,&acczqvect,&mytsQV,(qint64)mytsQV.at(0),(qint64)mytsQV.at(mytsQV.count()-1));
    TDVp->setGeometry(this->x()+this->width()+25, this->y()+30, TDVp->width(), TDVp->height());
    accxqvect.clear();
    accyqvect.clear();
    acczqvect.clear();
    mytsQV.clear();
    // At this point, input file is not needed anymore
    infilep->close();
    delete infilep;
    delete instrp;

    // check that there is actually usable data, if not exit
    if((!tsQVpp[0]) && (!tsQVpp[1]) && (!tsQVpp[2]) && (!tsQVpp[3]) &&
            (!tsQVpp[4]) && (!tsQVpp[5]) && (!tsQVpp[6]) && (!tsQVpp[7]))
    {
        msgBox.setText("No usable data in files... ");
        msgBox.exec();
        return false;   // user terminates so don't do anything else
    }

    return true;
}

void ReLax::closeEvent(QCloseEvent *)
{
    if(TDVp!=NULL)
    {
        delete TDVp;
        TDVp = 0;
    }
    QMessageBox myQMB;
    OPIPKT_t ucOpipkt;
    quint8 ucusdStatus;

    timer.stop();
    // if was running, close up things cleanly
    if(ui->startPB->isChecked())
    {
        ui->startPB->setChecked(false);
        on_startPB_clicked(false);
    }

    // move controller to off
    if(comPort == 0)
    {
        if(opi_openuce_com(&comPort))
        {
            qDebug() << "unable to open com port/set off mode when exiting";
        }
        else
        {
            opiuce_status(&comPort, &ucOpipkt);
            ucusdStatus = ucOpipkt.payload[DSNLEN+TSLEN+6+FWVLEN+1+PDNLISTLEN+1];
            if(!(ucusdStatus & 0x01))  // if tse not in
            {
                myQMB.setText("Please insert Sensor to turn off");
                myQMB.setStandardButtons(QMessageBox::Ok);
                myQMB.exec();
            }
            // try again to see if tse inserted or not
            opiuce_status(&comPort, &ucOpipkt);
            ucusdStatus = ucOpipkt.payload[DSNLEN+TSLEN+6+FWVLEN+1+PDNLISTLEN+1];
            if(opiuce_offmode(&comPort))
            {
                qDebug() << "unable to shutdown controller when exiting";
            }
            else
            {
                if(ucusdStatus & 0x01)  // if tse in
                {
                    myQMB.setText("Remove Sensor for low-power storage");
                    myQMB.setStandardButtons(QMessageBox::Ok);
                    myQMB.exec();
                }
            }
        }
        opi_closeuce_com(&comPort);
        comPort = 0;
    }
}

void ReLax::on_startPB_clicked(bool checked)
{
    OPIPKT_t ucOpipkt;
    QVector<qint64> tagTSQV;
    QVector<QString> tagTextQV;
    double beginOffFrms;
    QMessageBox msgQMB;
    QProgressDialog progQPD("Configuring", QString(), 0, ZBWLCHANCT);
    QString myQS;
    QFile *myoutQFp;
    QTextStream *outQTSp;

    progQPD.setWindowModality(Qt::WindowModal);
    progQPD.setMinimumDuration(10); // force it to open

    if(checked)
    {
        // disable inputs       ui->savedFileNameLa->setEnabled(false);
        ui->savedFilenameLE->setEnabled(false);
        ui->savedBrowsePB->setEnabled(false);
        ui->liveLevelLa->setEnabled(false);
        ui->liveLevelSli->setEnabled(false);
        ui->modeSli->setEnabled(false);
        ui->modeLiveLa->setEnabled(false);
        ui->modeSavedLa->setEnabled(false);
        // enable outputs
        ui->signalTimeSlider->setEnabled(true);
        ui->signalLine->setEnabled(true);
        ui->signalGV->setEnabled(true);
        ui->spectGV->setEnabled(true);
        ui->spectLBarLabel->setEnabled(true);
        ui->spectRBarLabel->setEnabled(true);
        ui->ledGV->setEnabled(true);

        // initialize stuff
        if(ui->modeSli->value() == LIVEMODE)     // do live mode stuff
        {
            noPktTimerCt = 0;

            ui->signalTimeSlider->setMaximum(MAXSHOWTIME);
            ui->signalTimeSlider->setMinimum(MINSHOWTIME);

            if(opi_openuce_com(&comPort))   // open com port, should open unless device was removed
            {
                ui->startPB->setChecked(false); // undo this
                opi_closeuce_com(&comPort);
                comPort = 0;
                return; // get out if this didn't open
            }
            if(setUCETime(&comPort))    // make sure time is right before starting
            {
                ui->startPB->setChecked(false); // undo this
                opi_closeuce_com(&comPort);
                comPort = 0;
                return; // get out if this didn't open
            }
            opiuce_settsertc(&comPort);  // propagate time set to tse
            if(opiuce_status(&comPort, &ucOpipkt))
            {
                ui->startPB->setChecked(false); // undo this
                opi_closeuce_com(&comPort);
                comPort = 0;
                return; // get out if this didn't open
            }

            if(ucOpipkt.payload[20] == 0xFF)    // if not paired, then get out
            {
                ui->startPB->setChecked(false); // undo this
                opi_closeuce_com(&comPort);
                comPort = 0;
                return; // get out if this didn't open
            }
            tsepdn = ucOpipkt.payload[20]; // only one

            // calculate algorithm parameters
            rlevel = (quint16) ui->liveLevelSli->value();
            thxx = THX;
            thmm = THM;
            thgm = THGM;
            thbm = THBM;
            offll = OFFL;
            offm = OFFM;

            //use rlevel 0~10 to calculate parameters: AttN invert from ReLax
            rlsm = 1.7 - ((float)rlevel)/10; //2.43X 0.7~1.7 for thm
            rls1 = 1.5 - ((float)rlevel)/12.5; //2.14X  0.7~1.5
            rls2 = rls1 * 1.5; //2*rls1
            rls3 = rls1 * 2.4; //3*rls1
            thx = thxx*rlsm; //M exclusion threshold
            thm = thmm*rlsm;
            offl = offll*rlsm;
            th3gm=thgm*rls3;
            th3bm=thbm*rls3;
            th2gm=thgm*rls2;
            th2bm=thbm*rls2;
            th1gm=thgm*rls1;
            th1bm=thbm*rls1;
            qDebug() << rlevel << thm << th1gm << th1bm << offl << offm << thx;            //if(opiuce_setrelaxparams(&comPort, THM, TH2GM, TH2BM, TH1GM, TH1BM, OFFL, OFF2GM, OFF2BM, OFF1GM, OFF1BM))
            if(opiuce_setrelaxparams(&comPort, thx, thm, offl, th3gm, th3bm, th2gm, th2bm, th1gm, th1bm, offm))
            {
                ui->startPB->setChecked(false); // undo this
                opi_closeuce_com(&comPort);
                comPort = 0;
                return; // get out if this didn't open
            }
            if(opiuce_getrelaxparams(&comPort, &ucOpipkt))
            {
                ui->startPB->setChecked(false); // undo this
                opi_closeuce_com(&comPort);
                comPort = 0;
                return; // get out if this didn't open
            }
            thx = (ucOpipkt.payload[1] << 8) + ucOpipkt.payload[2];
            thm = (ucOpipkt.payload[3] << 8) + ucOpipkt.payload[4];
            offl = (ucOpipkt.payload[5] << 8) + ucOpipkt.payload[6];
            th3gm = (ucOpipkt.payload[7] << 8) + ucOpipkt.payload[8];
            th3bm = (ucOpipkt.payload[9] << 8) + ucOpipkt.payload[10];
            th2gm = (ucOpipkt.payload[11] << 8) + ucOpipkt.payload[12];
            th2bm = (ucOpipkt.payload[13] << 8) + ucOpipkt.payload[14];
            th1gm = (ucOpipkt.payload[15] << 8) + ucOpipkt.payload[16];
            th1bm = (ucOpipkt.payload[17] << 8) + ucOpipkt.payload[18];
            offm = (ucOpipkt.payload[19] << 8) + ucOpipkt.payload[20];
//            qDebug() << rlevel << thm << th2gm << th2bm << offl << off2gm;

            // signal
            firstTimeDrawsignal = true;
            screenShowTime=INITIALADC; //sec
            ui->signalTimeSlider->setValue(screenShowTime);
            ui->signalTimeLa->setText(QString("%1s").arg(screenShowTime));
            screenmiddle=ui->signalGV->y()+(ui->signalGV->height())/2;
            signalmiddle = 0;
            signalDrawCount = 0;
            signalbegin=ui->signalGV->x();
            signalincrease=((float)ui->signalGV->width())/((float)screenShowTime*SAMPLERATEADC);
            scalerate=((float)(ui->signalGV->height()))/(float)(2*MAX_MIN_ADC); //ratio to max-min
            signalQGSp->clear();
            signalDataToDrawQQ.clear();
            // spectrogram
            spectDrawCount = 0;
            spectQGSp->clear();
            spectDataToDrawQV.clear();
            spectbegin=ui->spectGV->x();
            button=ui->spectGV->y()+ui->spectGV->height();
            numScalefft = LIVEFFTLEN/2; //256 bins, half of 256Hz BW
            // led
            ledQGSp->clear();
            ledDataToDrawQQ.clear();
            uceFFTDataToProcessQV.clear();
            ledDrawCount = 0;
            stRelax_pktCt = 0;
            stRelax_accum = 0;

            // posture viewer
            accxQV.clear();
            accyQV.clear();
            acczQV.clear();
            if(TDVp != NULL)
            {
                delete TDVp;
                TDVp = NULL;
            }
            TDVp = new twoDaccelviewer(true, QString("Posture Viewer [%1]").arg((int)tsepdn));
            TDVp->livedisplaylayout();
            TDVp->show();
            TDVp->setGeometry(this->x()+this->width()+25, this->y()+30, TDVp->width(), TDVp->height());
            qApp->processEvents(); // see if this makes posture viewer always draw

            // save to EDF file, clear variables
            // can't write header yet, because have to wait until get first
            // packet
            wroteDRCt = 0;
            // clear any stuff remaining from before
            tsQV.clear();
            skpQV.clear();
            batQV.clear();
            adcQV.clear();
            tmpQV.clear();
            axQV.clear();
            ayQV.clear();
            azQV.clear();
            sqQV.clear();
            edQV.clear();
        }
        else //SAVEDMODE
        {
            // do offline mode stuff
            //set message window title
            if(!ui->savedFilenameLE->text().isEmpty())
            {
                ui->signalTimePositionSlider->setEnabled(true);
                // check if file type correct
                if(whatInType(ui->savedFilenameLE->text()) == EDFEFILETYPE)
                {
                    index_begin =0;
                    signalstartindex=index_begin;
                    changablemiddle=ui->signalGV->y()+ui->signalGV->height()/2;
                    // read in data
                    convertedf();
                    qDebug()<<"setting signalTimeStart ";
                    ui->signalTimeStart->setText("0s");
                    qDebug()<<"setting signalTimeEnd";
                    ui->signalTimeEnd->setText(QString("%1s").arg(signalDataQvp.count()/SAMPLERATEADC));
                    qDebug()<<"setting signalTimePositionSlider";
                    ui->signalTimePositionSlider->setMaximum(signalDataQvp.count());

                    if((float)((float)signalDataQvp.count()/SAMPLERATEADC)>MAXSECONDINONESCENE)
                    {
                        ui->signalTimeSlider->setMaximum(MAXSECONDINONESCENE);
                        ui->signalTimeSlider->setMinimum(MINSECONDINONESCENE);
                        show_num_of_seconds=MAXSECONDINONESCENE; //same
                        ui->signalTimeSlider->setValue(show_num_of_seconds);
                        ui->signalTimeLa->setText(QString("%1s").arg(show_num_of_seconds));
                    }
                    else
                    {
                        ui->signalTimeSlider->setMaximum((float)signalDataQvp.count()/SAMPLERATEADC);//
                        ui->signalTimeSlider->setMinimum(MINSECONDINONESCENE);
                        show_num_of_seconds=(float)signalDataQvp.count()/SAMPLERATEADC; //same
                        ui->signalTimeSlider->setValue(show_num_of_seconds);
                        ui->signalTimeLa->setText(QString("%1s").arg(show_num_of_seconds));
                    }
                    //and draw
                    drawSavedSignal();
                    drawSavedSpect();
                    //TDVp->setVisible(true);
                    TDVp->show();
                }
                else
                {
                    msgQMB.setWindowTitle("Error");
                    msgQMB.setStyleSheet("background-image: url(:/images/bioshare-BGtiletexturedMixedGloss600px.jpg);");
                    msgQMB.setText("Unsupported File type");
                    msgQMB.exec();
                }
            }
        }
    }
    else
    {
        // disable outputs
        ui->signalTimeSlider->setEnabled(false);
        ui->savedFilenameLE->setEnabled(true);
        ui->signalLine->setEnabled(false);
        ui->signalGV->setEnabled(false);
        ui->spectGV->setEnabled(false);
        ui->spectLBarLabel->setEnabled(false);
        ui->spectRBarLabel->setEnabled(false);
        ui->ledGV->setEnabled(false);
        ui->signalTimePositionSlider->setEnabled(false);

        // enable inputs
        ui->savedBrowsePB->setEnabled(true);
        ui->liveLevelLa->setEnabled(true);
        ui->liveLevelSli->setEnabled(true);
        ui->modeSli->setEnabled(true);
        ui->modeLiveLa->setEnabled(true);
        ui->modeSavedLa->setEnabled(true);
        // deinitialize stuff
        if(ui->modeSli->value() == LIVEMODE)
        {
            // do live mode stuff
            opiuce_resetrelaxdata(&comPort);
            opi_closeuce_com(&comPort);
            comPort = 0;
            noPktTimerCt = 0;

//            signalQGSp-> setBrush(QColor(255, 255, 0, 127));
            signalQGSp->addText("AnyWhere mode: remove Controller\nOFF mode: insert Sensor, close APP", QFont("Arial", 13))
                    ->setPos(SIGNALMSGX, SIGNALMSGY);
            // if the file was opened for writing
            if(outQFp != 0)
            {
                // write tail of data, filling up datarecord so can be written
                if(tsQV.size() > 0)
                {
                    beginOffFrms = ((double) (stQDT.toMSecsSinceEpoch() % 1000))*TSERTCFREQ/1000/ADCLEN;
                    tagTSQV.append(tsQV.last()+ADCLEN*UCERTCFREQ/TSERTCFREQ+ADCLEN*UCERTCFREQ*beginOffFrms/TSERTCFREQ);
                    tagTextQV.append("DataEnd");

                    // put in default values for last record
                    while(tsQV.size() < 1*FRMSPERSEC*EDFDRDURSEC)
                    {
                        tsQV.append(tsQV.last()+ADCLEN*UCERTCFREQ/TSERTCFREQ);
                        skpQV.append(0);
                        batQV.append(1);
                        sqQV.append(0);
                        edQV.append(0);
                    }
                    while(adcQV.size() < ADCLEN*FRMSPERSEC*EDFDRDURSEC)
                        adcQV.append(0);
                    while(tmpQV.size() < TMPLEN*FRMSPERSEC*EDFDRDURSEC)
                        tmpQV.append(1024);   // temp ~25
                    while(axQV.size() < ACCLEN/4*FRMSPERSEC*EDFDRDURSEC)
                    {
                        axQV.append(0);
                        ayQV.append(0);
                    }
                    while(azQV.size() < ACCLEN*FRMSPERSEC*EDFDRDURSEC)
                        azQV.append(0);

                    procQV(&tsQV, &skpQV, &batQV, &adcQV,
                           &tmpQV, &axQV, &ayQV, &azQV, &sqQV, &edQV);
                    // write to file, add any tags that are within the timestamp ranges
                    wroteDRCt += edfEwrite(outQDSp, &adcQV, &tmpQV, &axQV, &ayQV, &azQV, &sqQV, wroteDRCt, firstFrmTS, &tsQV, &tagTSQV, &tagTextQV);
                }
                // rewrite header with correct number of data records
                outQFp->reset();
                edfEhdropiwrite(outQDSp, &localPatientID, &localRecordID, &stQDT, wroteDRCt);
                myQS = outQFp->fileName();
                outQFp->close();
                delete outQDSp;
                delete outQFp;
                outQDSp = NULL;
                outQFp = NULL;

                // write relax data to file if enough data
                if(stRelax_pktCt > RLSCOREMINPKTS)
                {
                    myQS.remove(myQS.size()-4,4).append(QString("_RL.txt")); // remove ".edf" and append "_RL.txt"
                    myoutQFp = new QFile(myQS);
                    if(myoutQFp->exists())
                    {
                        msgQMB.setWindowTitle("Out File Exists");
                        msgQMB.setText("Out file already exists in selected directory.");
                        msgQMB.setInformativeText("Do you want to overwrite?");
                        msgQMB.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
                        if(msgQMB.exec() == QMessageBox::Cancel)
                        {
                            delete myoutQFp;
                            return;
                        }
                    }
                    if(!myoutQFp->open(QIODevice::WriteOnly | QIODevice::Text))
                    {
                        delete myoutQFp;
                        return;
                    }
                    outQTSp = new QTextStream(myoutQFp);
//                    *outQTSp << "Accumulated RLScore " << stRelax_accum << endl;
                    *outQTSp << "Valid Duration(sec) " << (stRelax_pktCt/FRMSPERSEC) << endl;
                    *outQTSp << "Average RLScore " << QString("%1.%2").arg((qint32) (stRelax_accum/stRelax_pktCt)).arg((qint32) ((stRelax_accum*100/stRelax_pktCt) % 100),2,10,QChar('0')) << endl;

                    myoutQFp->close();
                    delete outQTSp;
                    delete myoutQFp;
                }
            }
        }
        this->setWindowTitle("ReLax");
    }
    qDebug() << "finished start";
}


void ReLax::on_savedBrowsePB_clicked()
{
    QString fileName = QFileDialog::getOpenFileName(this, tr("Open File"), QDir::currentPath(),
                       "Supported Files( *.edf ) ;; All Files( *.* )");
    if (!fileName.isEmpty())
    {
        ui->savedFilenameLE->setText(fileName);
    }
}


void ReLax::on_signalTimeSlider_valueChanged(int value)
{
    if(ui->modeSli->value() == SAVEDMODE)
    {
        show_num_of_seconds = value;
        ui->signalTimeLa->setText(QString("%1s").arg(value));  //save sec
        drawSavedSignal();
        drawSavedSpect();
    }
    else if(ui->modeSli->value() == LIVEMODE)
    {
        //Signal
        firstTimeDrawsignal = true;
        screenShowTime = value;
        signalDrawCount = 0;
        signalincrease=((float)ui->signalGV->width())/((float)screenShowTime*SAMPLERATEADC);
        signalQGSp->clear();
        ui->signalTimeLa->setText(QString("%1s").arg(value));   //save sec
    }
}


void ReLax::mousePressEvent(QMouseEvent *e)
{
    if( (ui->modeSli->value() == SAVEDMODE)&&(ui->startPB->isChecked()))
    {
        if(e->button()==Qt::LeftButton){
            if(e->x()>=ui->signalGV->x()){
                if(e->x()<=(ui->signalGV->x()+ui->signalGV->width())){
                    if(e->y()>=ui->signalGV->y()){
                        if(e->y()<=(ui->signalGV->y()+ui->signalGV->height())){
                            pressx=e->x();
                            pressy=e->y();
                            QCursor cursor(Qt::ClosedHandCursor);
                            this->setCursor(cursor);
                            this->grabMouse();
                            getmouseflag=1;
                        }//<=y width end
                    }//>=y end
                }//<=xwidth end
            }//>=x end
        }//left end
    }
}


void ReLax::mouseReleaseEvent(QMouseEvent *r)
{
    if((ui->modeSli->value() == SAVEDMODE)&&(ui->startPB->isChecked()))
    {
        QCursor cursor(Qt::ArrowCursor);
        int mousesignalincreasex=0,mousesignalincreasey=0;  //initial 0
        this->setCursor(cursor);
        if(getmouseflag==1){
            releasex=r->x();
            releasey=r->y();
            mousesignalincreasex=pressx-releasex;
            mousesignalincreasey=pressy-releasey;
            if((mousesignalincreasex*MOUSEINDEXACCURACYRATIO*ui->signalTimeSlider->value()>=MOUSEINDEXMOVETHRESHOLD)||(mousesignalincreasex*MOUSEINDEXACCURACYRATIO*ui->signalTimeSlider->value()<=-1*MOUSEINDEXMOVETHRESHOLD))
            {
                index_begin=index_begin+mousesignalincreasex*MOUSEINDEXACCURACYRATIO*ui->signalTimeSlider->value();
                if(index_begin<0)
                    index_begin=0;
                else if(index_begin>signalDataQvp.size())
                    index_begin=signalDataQvp.size();
            }
            if((mousesignalincreasey*MOUSEMIDDLEACCURACYRATIO>=MOUSEMIDDLEMOVETHRESHOLD)||(mousesignalincreasey*MOUSEMIDDLEACCURACYRATIO<=-1*MOUSEMIDDLEMOVETHRESHOLD))
            {
                changablemiddle-=(mousesignalincreasey*MOUSEMIDDLEACCURACYRATIO);
            }
            this->releaseMouse();
            getmouseflag=-1;
            drawSavedSignal();
            drawSavedSpect();
        }
    }
}



void ReLax::on_liveLevelSli_valueChanged(int value)
{
    ui->liveLevelLa->setText(QString("Level=%1").arg(value));
}


void ReLax::on_signalGainUpPB_clicked()
{
    signalZoomRatio *= 1.4;
    if(signalZoomRatio > SIGNALZOOMRATIOMAX)
        signalZoomRatio = SIGNALZOOMRATIOMAX;
    if(ui->modeSli->value() == LIVEMODE)
    {
        signalDrawCount = 0;
        signalQGSp->clear();
    }
    else if(ui->modeSli->value() == SAVEDMODE)
    {
        signalQGSp->clear();
        drawSavedSignal();
    }
}


void ReLax::on_signalGainDnPB_clicked()
{
    signalZoomRatio /= 1.4;
    if(signalZoomRatio < SIGNALZOOMRATIOMIN)
        signalZoomRatio = SIGNALZOOMRATIOMIN;
    if(ui->modeSli->value() == LIVEMODE)
    {
        signalDrawCount = 0;
        signalQGSp->clear();
    }
    else if(ui->modeSli->value() == SAVEDMODE)
    {
        signalQGSp->clear();
        drawSavedSignal();
    }
}

/*
void ReLax::on_getRelaxDataPB_clicked()
{

}
*/


void ReLax::on_spectEnablePB_clicked(bool checked)
{
    if(!checked)
    {
        ledQGSp->clear();
        spectQGSp->clear(); // clear screen
        spectDrawCount=0;   // start over
        ledDrawCount = 0;   // make sure led and spectrogram are always aligned
    }
}


void ReLax::on_GraphicsDrawCheckBox_clicked()
{
    if(ui->GraphicsDrawCheckBox->isChecked())
    {
        signalQGSp->clear();
        ledQGSp->clear();
        spectQGSp->clear();
        signalDrawCount = 0;
        ledDrawCount = 0;
        spectDrawCount = 0;
        if(TDVp != NULL)
        {
            delete TDVp;
            TDVp = NULL;
            //TDVp = new twoDaccelviewer(true,(quint8) tsepdn);
            TDVp = new twoDaccelviewer(true, QString("Posture Viewer [%1]").arg((int)tsepdn));
            TDVp->livedisplaylayout();
            //TDVp->setVisible(true);
            TDVp->show();
            TDVp->setGeometry(this->x()+this->width()+25, this->y()+30, TDVp->width(), TDVp->height());
        }

    }
}


void ReLax::on_signalTimePositionSlider_valueChanged(int position)
{
    ui->signalTimeStart->setText(QString("%1s").arg((int)((float)position/SAMPLERATEADC)));
    index_begin = position;
    drawSavedSignal();
    drawSavedSpect();
}



void ReLax::getRelaxData()
{
    OPIPKT_t relax1Opipkt, ucOpipkt;
    QFile *myoutQFp;
    QTextStream *outQTSp;
    qint64 startTS;
    QDateTime startQDT, refDT;
    quint8 mytsepdn;
    QMessageBox msgQMB;
    qint32 relSt_accum, relSt_pktCt;
    QString myQS;
    QTime killTime;

    // assumes port is open already
    if(opiuce_status(&comPort, &ucOpipkt))
    {
        killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
        while(QTime:: currentTime() < killTime);
        if(opiuce_status(&comPort, &ucOpipkt))
        {
            return; // get out if this didn't open
        }
    }
    mytsepdn = ucOpipkt.payload[DSNLEN+TSLEN+6+FWVLEN+1];
    if(opiuce_getrelaxdata(&comPort, &relax1Opipkt))
    {
        killTime = QTime::currentTime().addMSecs(COMRETRYTIME);
        while(QTime:: currentTime() < killTime);
        if(opiuce_getrelaxdata(&comPort, &relax1Opipkt))
        {
            return; // get out if this didn't open
        }
    }

    startTS = ((qint64) relax1Opipkt.payload[1] << 40) + ((qint64) relax1Opipkt.payload[2] << 32) +
            ((qint64) relax1Opipkt.payload[3] << 24) + ((qint64) relax1Opipkt.payload[4] << 16) +
            ((qint64) relax1Opipkt.payload[5] << 8) + ((qint64) relax1Opipkt.payload[6]);
    refDT = QDateTime::fromString("20120928080000000","yyyyMMddhhmmsszzz");
    startQDT = QDateTime::fromMSecsSinceEpoch(startTS*1000/UCERTCFREQ+refDT.toMSecsSinceEpoch());
    relSt_accum = ((qint32) relax1Opipkt.payload[7] << 24) + ((qint32) relax1Opipkt.payload[8] << 16) +
            ((qint32) relax1Opipkt.payload[9] << 8) + ((qint32) relax1Opipkt.payload[10]);
    relSt_pktCt = ((qint32) relax1Opipkt.payload[11] << 24) + ((qint32) relax1Opipkt.payload[12] << 16) +
            ((qint32) relax1Opipkt.payload[13] << 8) + ((qint32) relax1Opipkt.payload[14]);
    if(relSt_pktCt < RLSCOREMINPKTS) return;

    // write data to file
    myQS = QString("E%1_%2_RL.txt").arg(startQDT.toString("yyyyMMdd_hhmmss")).arg(mytsepdn);
    myoutQFp = new QFile(myQS);
    if(myoutQFp->exists())
    {
        msgQMB.setWindowTitle("Out File Exists");
        msgQMB.setText("Out file already exists in selected directory.");
        msgQMB.setInformativeText("Do you want to overwrite?");
        msgQMB.setStandardButtons(QMessageBox::Ok | QMessageBox::Cancel);
        if(msgQMB.exec() == QMessageBox::Cancel)
        {
            delete myoutQFp;
            return;
        }
    }
    if(!myoutQFp->open(QIODevice::WriteOnly | QIODevice::Text))
    {
        delete myoutQFp;
        return;
    }
    outQTSp = new QTextStream(myoutQFp);
//    *outQTSp << "Accumulated RLScore " << relSt_accum << endl;
    *outQTSp << "Valid Duration(sec) " << (relSt_pktCt/FRMSPERSEC) << endl;
    *outQTSp << "Average RLScore " << QString("%1.%2").arg((qint32) (relSt_accum/relSt_pktCt)).arg((qint32) ((relSt_accum*100/relSt_pktCt) % 100),2,10,QChar('0')) << endl;

    myoutQFp->close();
    delete outQTSp;
    delete myoutQFp;

    // display data to screen
    ui->RLScoreLa->setText(QString("RL%1.%2").arg(relSt_accum/relSt_pktCt).arg((relSt_accum*100)/relSt_pktCt-(relSt_accum/relSt_pktCt)*100));
}
