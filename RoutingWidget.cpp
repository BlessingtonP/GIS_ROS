#include "RoutingWidget.h"
#include "ui_routingwidget.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QThread>
#include <QFileInfo>
#include <gdal_priv.h>
#include <ogrsf_frmts.h>

#include "RoutingWorker.h"
#include "MapLoad/maploadmodule.h"
#include <FileSystemModel/filesystemmodeldialog.h>
#include <App/NotificationBox.h>
#include <QDateTime>

RoutingWidget::RoutingWidget(AppMapView *rcMapView, osgEarth::MapNode* mapNode)
    : QWidget(rcMapView)
    , ui(new Ui::RoutingWidget), m_mapNode(mapNode)
{
    ui->setupUi(this);

    m_mapView = rcMapView;

    m_txLocationDialog = new LocationDialog(m_mapView,
                                            CGPointStyle("RadioTx.png", ":/icons/Analysis/",
                                                         CGColor(), 30, CGStyle::Mode_ConstSymSize,
                                                         CGPointTypeStyle::Origin_BottomMid),false,false);

    m_txLocationDialog->setHeightAGL(4.5);


    QVBoxLayout *txLayout = new QVBoxLayout(this);

    txLayout->addWidget(m_txLocationDialog);

    ui->txLocWidget->setLayout(txLayout);

    m_rxLocationDialog = new LocationDialog(m_mapView,
                                            CGPointStyle("RadioTx.png", ":/icons/Analysis/",
                                                         CGColor(), 30, CGStyle::Mode_ConstSymSize,
                                                         CGPointTypeStyle::Origin_BottomMid),false,false);


    m_rxLocationDialog->setHeightAGL(4.5);

    QVBoxLayout *rxLayout = new QVBoxLayout(this);

    rxLayout->addWidget(m_rxLocationDialog);

    ui->rxLocWidget->setLayout(rxLayout);

    connect(ui->browseButton, &QPushButton::clicked, this, &RoutingWidget::selectShapefile);
    connect(ui->computeButton, &QPushButton::clicked, this, &RoutingWidget::computePath);
    connect(ui->resetButton, &QPushButton::clicked, this, &RoutingWidget::resetFields);

    m_loadingWidget = new LoadingWidget(this);

    ui->verticalLayout_LoadingWidget->addWidget(m_loadingWidget);
}

RoutingWidget::~RoutingWidget()
{
    delete ui;
}

void RoutingWidget::selectShapefile()
{
    QString filePath;

    FileSystemModelDialog fileDialog(this, "Select Image 1", QDir::homePath(), true);

    fileDialog.setNameFilters(QStringList() << "*.shp"); // Filter for image files
    fileDialog.showFileOpenDlg();

    QStringList selectedFiles = fileDialog.fileNames();

    if (!selectedFiles.isEmpty())
    {

        filePath = selectedFiles.first();
    }
    if (!filePath.isEmpty())
        validateShapefile(filePath);
}

void RoutingWidget::validateShapefile(const QString &path)
{
    ui->logTextEdit->clear();
    ui->logTextEdit->appendPlainText("Opening shapefile: " + path);

    GDALAllRegister();
    GDALDataset *ds = (GDALDataset*)GDALOpenEx(
                path.toStdString().c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr);
    if (!ds) {
        QMessageBox::critical(this, "Error", "Failed to open shapefile.");
        ui->logTextEdit->appendPlainText("ERROR: could not open file.");
        return;
    }

    OGRLayer *layer = ds->GetLayer(0);
    if (!layer || wkbFlatten(layer->GetGeomType()) != wkbLineString) {
        QMessageBox::critical(this, "Error", "Selected shapefile is not Line geometry.");
        ui->logTextEdit->appendPlainText("ERROR: wrong geometry type.");
        GDALClose(ds);
        return;
    }

    shapefilePath = path;
    ui->lineEdit_inputFile->setText(path);
    ui->logTextEdit->appendPlainText("Shapefile validated.");
    GDALClose(ds);
}

void RoutingWidget::computePath()
{
    if (shapefilePath.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please select a valid shapefile first.");
        return;
    }

    bool bTxLocValid, bRxLocValid;

    m_TxLoc = m_txLocationDialog->getLocation(&bTxLocValid);

    m_RxLoc = m_rxLocationDialog->getLocation(&bRxLocValid);

    if(!bTxLocValid || !bRxLocValid)
    {
        NotificationBox::showMessage("Select both Start and End Locations!",
                                     QFont("Poppins", 12),
                                     2000, // time interval to destroy after
                                     m_mapView,
                                     QColor(2,59,89));
        return;
    }
    bool ok1, ok2, ok3, ok4, ok5, ok6, ok7;
    double lat1 = m_TxLoc.y();
    double lon1 = m_TxLoc.x();
    double alt1 = m_txLocationDialog->getElevation();
    double lat2 = m_RxLoc.y();
    double lon2 = m_RxLoc.x();
    double alt2 = m_rxLocationDialog->getElevation();

    double sx = lon1;
    double sy = lat1;
    double ex = lon2;
    double ey = lat2;

    m_loadingWidget->startLoading();

    runRoutingInThread(sx, sy, ex, ey);
}

void RoutingWidget::runRoutingInThread(double sx, double sy, double ex, double ey)
{
    ui->computeButton->setEnabled(false);
    ui->label_Length->clear();

    ui->logTextEdit->appendPlainText("\n"+QDateTime::currentDateTime().toString("hh:mm:ss:zzz - ")+"Starting routing thread...");

    QThread *thread = new QThread;

    RoutingWorker *worker = nullptr;
    if(ui->radioButton_Dijkstras->isChecked())
    {
        worker = new RoutingWorker(shapefilePath, sx, sy, ex, ey,RoutingWorker::Dijkstra, m_mapNode);
    }
    else if(ui->radioButton_AStar->isChecked())
    {
        worker = new RoutingWorker(shapefilePath, sx, sy, ex, ey,RoutingWorker::AStar, m_mapNode);
    }
    worker->moveToThread(thread);

    connect(thread, &QThread::started, worker, &RoutingWorker::process);
    connect(worker, &RoutingWorker::stageUpdate, this, &RoutingWidget::appendLog);
    connect(worker, &RoutingWorker::routingComplete, this, &RoutingWidget::onRoutingFinished);
    connect(worker, &RoutingWorker::finished, thread, &QThread::quit);
    connect(worker, &RoutingWorker::finished, worker, &RoutingWorker::deleteLater);
    connect(thread, &QThread::finished, thread, &QThread::deleteLater);

    if(!prevOP.isEmpty())
    {
        // Wrap in a QFileInfo
        QFileInfo fi(prevOP);

        // 1) Get just the filename without its extension:
        QString nameOnly = fi.completeBaseName();
        // — for "route_dijkstra.shp" this yields "route_dijkstra"

        // 2) If you want the full absolute path *up to* that basename:
        QString dir      = fi.absolutePath();                   // "/home/user/data"
        QString fullBase = dir + QDir::separator() + nameOnly;  // "/home/user/data/route_dijkstra"

        // 3) If you only wanted the name (no path), use nameOnly directly:
        qDebug() << "Base name:" << nameOnly;

        CGLegend *legend = m_mapView->legend();
        CGLegendEntry *le = legend->item(nameOnly);
        if (le)
        {
            le->setVisible(false);
            m_mapView->repaintMapForce(m_mapView->currentViewExtent());
        }
    }

    thread->start();
}

void RoutingWidget::appendLog(const QString &msg)
{
    ui->logTextEdit->appendPlainText(QDateTime::currentDateTime().toString("hh:mm:ss:zzz - ")+msg);
    if(msg.contains("Path length"))
    {
        ui->label_Length->setText(msg);
    }
}

void RoutingWidget::onRoutingFinished(const QString &outPath)
{
    ui->computeButton->setEnabled(true);
    ui->logTextEdit->appendPlainText("Routing complete. Output: " + outPath);

    QStringList list = outPath.split(";");
    if(list.count()>1)
    {
        MapLoadModule::getInstance()->openMap(list.at(0));

        prevOP = list.at(0);
    }
    m_loadingWidget->stopLoading();

    QMessageBox::information(this, "Done", "Saved to:" + outPath);
}

void RoutingWidget::resetFields()
{
    shapefilePath.clear();
    m_txLocationDialog->reset();
    m_rxLocationDialog->reset();
    ui->logTextEdit->clear();
}
