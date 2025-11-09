#include "udp_receiver_widget.h"
#include "ui_udp_receiver_widget.h"
#include <QHostAddress>
#include <QRegularExpression>
#include <QRegularExpressionMatch>
#include <3DMapView/threedmapscene.h>

UdpReceiverWidget::UdpReceiverWidget(AppMapView* rcMapView)
    : QWidget(rcMapView),
      ui(new Ui::UdpReceiverWidget),
      m_mapView(rcMapView),
      socket(new QUdpSocket(this))
{
    ui->setupUi(this);

    // Defaults
    ui->ipEdit->setText("0.0.0.0");
    ui->portSpin->setRange(1, 65535);
    ui->portSpin->setValue(5000);

    connect(ui->clearBtn, &QPushButton::clicked, this, &UdpReceiverWidget::clearData);
    setudpinitializeVectorLayer();

    auto mapNode = ThreeDMapScene::getInstance()->getSceneManager()->getMapNode();

    m_3DPointHandler = new ThreedPointHandler(mapNode);
    ui->spinBox_bulk->setRange(1,1000);
    ui->spinBox_bulk->setValue(10);
    connect(ui->spinBox_bulk, QOverload<int>::of(&QSpinBox::valueChanged), this, [=](int value){
        m_bulkInsertThreshold = value;
    });

    m_mapNode = mapNode;

    Add_location();
}

UdpReceiverWidget::~UdpReceiverWidget()
{
    delete ui;
}

void UdpReceiverWidget::setudpinitializeVectorLayer()
{
    m_udpvectorPipe = new CGVectorPipe("UDPTrackPipe", "UDPTrackPipe", CGCoordinateSystemMgr::GeoCS_WGS84);
    if(!m_mapView->mapRepository()->addPipe(m_udpvectorPipe)){
        qDebug()<<"Failed to add vector pipe to map repository";
    }
    else
        qDebug()<<"Vector pipe added to map repository successfully";

    if(!m_udpvectorPipe){
        qDebug()<<"Failed to initialize vector pipe";
        return;
    }
    else{
        qDebug()<<"Vector pipe initialized successfully";
    }
    QString layerName = "UDPTrackLayer";
    CGTableDef pointFeatureLayer;
    pointFeatureLayer.setTableName(layerName);
    pointFeatureLayer.append(CGFieldDef("ID", "Feature ID", CGFieldDef::Type_Integer));
    pointFeatureLayer.append(CGFieldDef("Longitude", "Longitude", CGFieldDef::Type_Double));
    pointFeatureLayer.append(CGFieldDef("Latitude", "Latitude", CGFieldDef::Type_Double));
    pointFeatureLayer.append(CGFieldDef("Altitude", "Altitude", CGFieldDef::Type_Double));
    CGPointStyle pointStyle("", "", CGColor(255,0,0),12);
    pointStyle.setZoomMode(CGStyle::Mode_ConstSymSize);
    CGFeatureLayerDef pointLayerDef(layerName, "UDP Track Layer", CGGeometry::Type_Point, &pointStyle, pointFeatureLayer);
    if(!m_udpvectorPipe->insertLayer(pointLayerDef)){
        qDebug()<<"Failed to create the layer:" <<layerName;
        return;
    }
    else{
        qDebug()<<"Layer created successfully:"<<layerName;
    }
//    CGPointer<CGFeatureLayer> layer = m_udpvectorPipe->getLayer(layerName);
    m_udpTrackLayer = m_udpvectorPipe->getLayer(layerName);
    if (!m_udpTrackLayer.isNull())
    {
        CGTextStyle* textStyle = new CGTextStyle("Arial", 14.0f, false, false, false, CGStyle::Mode_ConstSymSize);
        textStyle->setColor(CGColor(0, 0, 0));
        QString labelFormat = "%0";
        CGFeatureLabel* label = new CGFeatureLabel(labelFormat, textStyle,
                                                   CGFeatureLabel::Position_Center,
                                                   5, 5, 0.0);
        m_udpTrackLayer->setLabel(label);
        qDebug() << "Label enabled for layer " << layerName;
    }
    else
    {
        qDebug() << "Failed to get layer for labeling: " << layerName;
    }
}

//THIS IS THE ACTUAL CODE

//void UdpReceiverWidget::onReadyRead()
//{
//    QRegularExpression re(R"(ID:(\d+),Longitude:([-\d\.]+),Latitude:([-\d\.]+),Altitude:([-\d\.]+))");
//    QList<Track> validTracks;
//    CGFeatureObjectCollection collection;
//    while (socket->hasPendingDatagrams())
//    {
//        QByteArray datagram;
//        datagram.resize(int(socket->pendingDatagramSize()));
//        QHostAddress sender;
//        quint16 senderPort;
//        socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

//        qDebug()<<"INDIGIS Received packet: "<<QString(datagram);

//        QRegularExpressionMatch match = re.match(QString::fromUtf8(datagram));
//        if (match.hasMatch())
//        {
//            Track t;
//            t.id = match.captured(1).toInt();
//            t.longitude = match.captured(2).toDouble();
//            t.latitude = match.captured(3).toDouble();
//            t.altitude = match.captured(4).toDouble();
//            tracks.append(t);
//            m_trackBuffer.append(t);

//            /*  ui->listWidget->addItem(*/
//            QString displayText = QString("ID:%1, Lon:%2, Lat:%3, Alt:%4 from %5:%6")
//                    .arg(t.id)
//                    .arg(t.longitude, 0, 'f', 6)
//                    .arg(t.latitude, 0, 'f', 6)
//                    .arg(t.altitude, 0, 'f', 2)
//                    .arg(sender.toString())
//                    .arg(senderPort);
//            ui->listWidget->addItem(displayText);
//                if(m_idToListitem.contains(t.id)){
//                    m_idToListitem[t.id]->setText(displayText);
//            } else {
//                    QListWidgetItem* item = new QListWidgetItem(displayText,ui->listWidget );
//                    m_idToListitem[t.id] = item;
//            }
//            validTracks.append(t);
//        } else {
//            ui->listWidget->addItem("Invalid: " + datagram);
//        }
//    }
//    // Check if threshold reached
//    int maxBufferSize = ui->spinBox_bulk->value();
//    if (m_trackBuffer.size() >= maxBufferSize) {
////        CGFeatureObjectCollection collection;
//        qDebug()<<"Inserting" <<m_trackBuffer.size()<<"tracks to map(SpinBox value: "<<maxBufferSize<<")";
//        insertMultipleTracksToLayer(m_trackBuffer, collection);
//        m_trackBuffer.clear();  // Clear after inserting
//    }

//    if (!m_udpTrackLayer.isNull()) {
//        ui->label->setText(QString("Tracks on Map: %1 | Buffered: %2 / %3")
//            .arg(m_udpTrackLayer->count())
//            .arg(m_trackBuffer.size())
//            .arg(ui->spinBox_bulk->value()));
//    }

//}


#include <QtEndian>

void UdpReceiverWidget::onReadyRead()
{
    QHostAddress sender;
    quint16 senderPort;

    while (socket->hasPendingDatagrams())
    {
        QByteArray datagram;
        datagram.resize(int(socket->pendingDatagramSize()));
        socket->readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        qDebug() << "Received datagram from" << sender.toString() << ":" << senderPort
<< "Size:" << datagram.size();

        if (datagram.size() < 256) {
            ui->listWidget->addItem("Received datagram too small: " + QString::number(datagram.size()));
            continue;
        }

        NavMessage nav;
        std::string parseError;
        bool success = parse_nav(reinterpret_cast<const uint8_t*>(datagram.constData()), datagram.size(), nav, parseError);

        if (!success) {
            ui->listWidget->addItem(QString("Parse error: %1").arg(QString::fromStdString(parseError)));
            continue;
        }

        // If parse is successful, Update_location is already called inside parse_nav()
        // Optionally, you can also show the data in the UI:
        QString displayText = QString("NAV: Lat: %1, Lon: %2, Alt: %3")
            .arg(nav.lat_deg, 0, 'f', 6)
            .arg(nav.lon_deg, 0, 'f', 6)
            .arg(nav.alt_m, 0, 'f', 2);

        ui->listWidget->addItem(displayText);
    }
}


bool UdpReceiverWidget::parse_nav(const uint8_t* buf, size_t len, NavMessage& out, std::string& err)
{

  if (len < 256) {
    err = "buffer too small";
    return false;
  }

  // Header
  if (!(buf[0] == 'N' && buf[1] == 'A' && buf[2] == 'V')) {
    err = "bad tag";
    return false;
  }
  out.tag[0] = 'N';
  out.tag[1] = 'A';
  out.tag[2] = 'V';
  out.version = buf[3];
  out.total_len = be16(buf + 4);
  if (out.total_len != 256) {
    err = "unexpected total_len";
    return false;
  }
  out.message_type = buf[6];
  if (out.message_type != 0x03) {
    err = "unexpected message_type";
    return false;
  }
  out.flags = buf[7];

  // Core
  out.sequence = be32(buf + 8);
  out.epoch_seconds = be32(buf + 12);
  out.epoch_ms_residual = be32(buf + 16);

  // Endpoints
  out.source_ipv4_nbo = be32(buf + 20);
  out.source_port = be16(buf + 24);
  out.dest_ipv4_nbo = be32(buf + 26);
  out.dest_port = be16(buf + 30);

  // Timestamp
  out.year = be16(buf + 32);
  out.month = buf[34];
  out.day = buf[35];
  out.hour = buf[36];
  out.minutes = buf[37];
  out.seconds = buf[38];
  out.milliseconds = be16(buf + 39);

  // Attitude, rates, heave
  out.roll_deg = be_float(buf + 41);
  out.pitch_deg = be_float(buf + 45);
  out.heading_deg = be_float(buf + 49);
  out.roll_rate_dps = be_float(buf + 53);
  out.pitch_rate_dps = be_float(buf + 57);
  out.heading_rate_dps = be_float(buf + 61);
  out.heave_m = be_float(buf + 65);

  // Vel NED
  out.vel_n_mps = be_float(buf + 69);
  out.vel_e_mps = be_float(buf + 73);
  out.vel_d_mps = be_float(buf + 77);

  // Geodetic
  out.lat_deg = be_double(buf + 81);
  out.lon_deg = be_double(buf + 89);
  out.alt_m = be_float(buf + 97);

  // Std devs
  out.lat_std = be16(buf + 101);
  out.lon_std = be16(buf + 103);
  out.vel_n_std = be16(buf + 105);
  out.vel_e_std = be16(buf + 107);
  out.vel_d_std = be16(buf + 109);
  out.roll_std = be16(buf + 111);
  out.pitch_std = be16(buf + 113);
  out.heading_std = be16(buf + 115);

  // Altimeter/env
  out.altimeter_depth_m = be_float(buf + 117);
  out.altimeter_temp_c = be_float(buf + 121);
  out.running_depth_mm = be16(buf + 125);
  out.reserved1 = be16(buf + 127);  // expect 0

  // Mission
  out.mission_cmd = buf[129];
  out.reserved2 = buf[130];  // expect 0

  out.x_m = be_float(buf + 131);
  out.y_m = be_float(buf + 135);
  out.z_m = be_float(buf + 139);
  out.surge_mps = be_float(buf + 143);
  out.sway_mps = be_float(buf + 147);

  Update_location(out.lat_deg, out.lon_deg, out.alt_m);

  return true;
}


void UdpReceiverWidget::Update_location(double lat_deg, double lon_deg, double alt_m)
{
//    std::cout << "[DEBUG] lat = " << lat_deg << ", lon = " << lon_deg << ", alt = " << alt_m << std::endl;

//    std::cout << "[DEBUG] BEFORE GeoPOINT Update_location()" << std::endl;

    if (!m_mapNode)
    {
        std::cout << "[FATAL] m_mapNode is nullptr!" << std::endl;
        return;
    }
    else
    {
//        std::cout << "[DEBUG] m_mapNode is valid." << std::endl;
    }

    osgEarth::GeoPoint point(m_mapNode->getMapSRS(), lon_deg , lat_deg, alt_m, osgEarth::ALTMODE_RELATIVE);

//    std::cout << "[DEBUG] AFTER GeoPOINT Update_location()" << std::endl;

    // Append to path
    if (_flightPath.valid())
    {
        _flightPath->push_back(point.vec3d());
//        std::cout << "[DEBUG] Point added to _flightPath: " << point.vec3d().x() << ", "
//                  << point.vec3d().y() << ", " << point.vec3d().z() << std::endl;
    }
    else
    {
        std::cout << "[WARN] _flightPath is not valid!" << std::endl;
    }

//    std::cout << "[DEBUG] Total points in _flightPath: " << _flightPath->size() << std::endl;

    if (!_flightPathNode.valid())
    {
//        std::cout << "[WARN] _flightPathNode is not valid!" << std::endl;
        return;
    }
    else
    {
//        std::cout << "[DEBUG] _flightPathNode is valid!" << std::endl;
    }

    auto feature1 = _flightPathNode->getFeature();
    if (!feature1)
    {
 //       std::cout << "[WARN] _flightPathNode->getFeature() returned nullptr!" << std::endl;
        return;
    }
    if (feature1)
    {
        feature1->setGeometry(_flightPath);
        _flightPathNode->dirty();
 //       std::cout << "[DEBUG] Feature geometry updated and _flightPathNode marked dirty." << std::endl;
    }
    else
    {
//        std::cout << "[WARN] _flightPathNode has no valid feature!" << std::endl;
    }
}


void UdpReceiverWidget::Add_location()
{
    _flightPath = new osgEarth::LineString();

    osg::ref_ptr<osgEarth::Feature> feature = new osgEarth::Feature(_flightPath.get(), m_mapNode->getMapSRS());
    feature->geoInterp() = osgEarth::GEOINTERP_RHUMB_LINE;

    osgEarth::Style style;
    auto* line = style.getOrCreate<osgEarth::LineSymbol>();
    line->stroke()->color() = osgEarth::Color::Red;
    line->stroke().mutable_value().width() = osgEarth::Distance(5.0,osgEarth::Units::PIXELS);


    auto* alt = style.getOrCreate<osgEarth::AltitudeSymbol>();
    alt->clamping() = osgEarth::AltitudeSymbol::CLAMP_NONE;
    alt->technique() = osgEarth::AltitudeSymbol::TECHNIQUE_GPU;

   _flightPathNode = new osgEarth::FeatureNode(feature.get(), style);
    _flightPathNode->setNodeMask(0xffffffff);

    group = new osg::Group();

    group->addChild(_flightPathNode.get());

    m_mapNode->addChild(group);
}

void UdpReceiverWidget::clearData()
{
    int totalPackets = ui->listWidget->count();
    ui->label->setText(QString("Total packets received before stop: %1").arg(totalPackets));
    ui->label->setVisible(true);
    tracks.clear();
    ui->listWidget->clear();
    m_idToListitem.clear();
    if (!m_udpvectorPipe)
        return;
    CGPointer<CGFeatureLayer> udpLayer = m_udpvectorPipe->getLayer("UDPTrackLayer");
    if (udpLayer.isNull()) {
        qDebug() << "UDP layer not found during clear";
        return;
    }
    udpLayer->clearAllObjects();
    m_mapView->update();
    m_mapView->refresh();
}
bool UdpReceiverWidget::insertOrUpdateUdpTrackonMap(int id, double latitude, double longitude, double altitude)
{
    if (!m_udpvectorPipe) {
        qDebug() << "Vector pipe is not initialized";
        return false;
    }
    if (latitude < -90.0 || latitude > 90.0 || longitude < -180.0 || longitude > 180.0) {
        qDebug() << "Invalid geographic coordinates";
        return false;
    }
    if(m_udpTrackLayer.isNull()){
        qDebug()<<"UDP track layer is null";
        return false;
    }
    CGFeatureObject obj;
    if (m_idToFeatureObject.contains(id)) {
        obj = m_idToFeatureObject[id];
    } else {
        obj = m_udpTrackLayer->createFeatureObject(id);
        if (obj.isNull()) {
        }
        m_idToFeatureObject[id] = obj;
    }
    CGRecord rec = obj.featureAttribRecord();
    rec.setFieldValue("ID", id);
    rec.setFieldValue("Longitude", longitude);
    rec.setFieldValue("Latitude", latitude);
    rec.setFieldValue("Altitude", altitude);
    obj.setFeatureAttribRecord(rec);
    CGPointGeom pointGeom(CGPointF(longitude, latitude));
    if (!obj.setFeatureGeometry(&pointGeom)) {
        qDebug() << "Failed to set geometry";
        return false;
    }
    int result = m_udpvectorPipe->insertOrUpdateFeatureObject(obj);
    if (result < 0) {
        qDebug() << "Failed to insert or update feature object";
        return false;
    }
    m_mapView->refresh();

    ////Implement insert and update 3D point on map here

    if(m_3DPointHandler)
    {
        m_3DPointHandler->createorUpdate3DPoint(id,longitude,latitude,altitude);
    }
    //Implement insert and update 3D point on map here

    return true;
}
void UdpReceiverWidget::updateTotalPacketCount(){
    int total = ui->listWidget->count();
    ui->label->setText(QString("Total packets: %1").arg(total));
    ui->label->setVisible(true);
}

void UdpReceiverWidget::on_bindBtn_clicked(bool checked)
{
    if(checked)
    {
        if (isBound) {
            socket->close();
            disconnect(socket, &QUdpSocket::readyRead, this, &UdpReceiverWidget::onReadyRead);
            isBound = false;
            ui->listWidget->addItem("Unbound previous socket.");
            //            updateTotalPacketCount();
        }

        QHostAddress addr;
        if (!addr.setAddress(ui->ipEdit->text())) {
            ui->listWidget->addItem("Invalid IP address!");
            return;
        }
        quint16 port = static_cast<quint16>(ui->portSpin->value());

        if (!socket->bind(addr, port, QUdpSocket::ShareAddress)) {
            ui->listWidget->addItem("Failed to bind to " + ui->ipEdit->text() + ":" + QString::number(port));
            return;
        }

        connect(socket, &QUdpSocket::readyRead, this, &UdpReceiverWidget::onReadyRead, Qt::UniqueConnection);
        isBound = true;
        ui->listWidget->addItem("Listening on UDP " + ui->ipEdit->text() + ":" + QString::number(port));

        ui->bindBtn->setText("UNBIND");

        if(m_3DPointHandler)
            m_3DPointHandler->clearPoints();
    }
    else
    {
        if (isBound) {
            socket->close();
            disconnect(socket, &QUdpSocket::readyRead, this, &UdpReceiverWidget::onReadyRead);
            isBound = false;
            ui->listWidget->addItem("Unbound previous socket.");
            ui->label->clear();
            //            updateTotalPacketCount();
        }

        ui->bindBtn->setText("BIND");
    }
}
void UdpReceiverWidget::insertMultipleTracksToLayer(const QList<Track>& tracks, CGFeatureObjectCollection& collection)
{
    if (!m_udpvectorPipe) {
        qDebug() << "Vector pipe is not initialized";
        return;
    }

    if (m_udpTrackLayer.isNull()) {
        qDebug() << "UDP layer is null";
        return;
    }
    for (const Track& t : tracks)
    {
        if (t.latitude < -90.0 || t.latitude > 90.0 || t.longitude < -180.0 || t.longitude > 180.0) {
            qDebug() << "Invalid coordinates for ID:" << t.id;
            continue;
        }
        CGFeatureObject obj;
        if (m_idToFeatureObject.contains(t.id)) {
            obj = m_idToFeatureObject[t.id];
        } else {
            obj = m_udpTrackLayer->createFeatureObject(t.id);
            if (obj.isNull()) {
            }
            m_idToFeatureObject[t.id] = obj;
        }
        CGRecord rec = obj.featureAttribRecord();
        rec.setFieldValue("ID", t.id);
        rec.setFieldValue("Longitude", t.longitude);
        rec.setFieldValue("Latitude", t.latitude);
        rec.setFieldValue("Altitude", t.altitude);
        obj.setFeatureAttribRecord(rec);
        CGPointGeom pointGeom(CGPointF(t.longitude, t.latitude));
        if (!obj.setFeatureGeometry(&pointGeom)) {
            qDebug() << "Failed to set geometry for ID:" << t.id;
            continue;
        }
        int result = m_udpvectorPipe->insertOrUpdateFeatureObject(obj);
        if (result < 0) {
            qDebug() << "Insert/update failed for ID:" << t.id;
        }
        collection.append(obj);

        ////Implement insert and update 3D point on map here

        if(m_3DPointHandler)
        {
            m_3DPointHandler->createorUpdate3DPoint(t.id,t.longitude,t.latitude,t.altitude);
        }
        ////Implement insert and update 3D point on map here
    }
    m_mapView->refresh();
    qDebug()<<"Track count:"<<m_udpTrackLayer->count();
}
