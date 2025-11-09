#pragma once

#include <QWidget>
#include <QUdpSocket>
#include <QList>
#include <QMap>
#include <QListWidget>
#include <CGMapView/cgmaprepository.h>
#include <MainWindow/appmapview.h>
#include <CGPipe/cgvectorpipe.h>
#include "CGFeature/cgfeaturelayerdef.h"
#include "CGFeature/cgfeaturelayer.h"
#include "CGFeature/cgfeatureobject.h"
#include "threedpointhandler.h"
#include "Tools/RouteFinder/RoutingWorker.h"
#include <osgEarth/MapNode>
#include <osg/Group>
//osgEarth libraries

#include <osg/Group>
#include <osg/Node>
#include <osgDB/ReadFile>
#include <osgEarth/MapNode>
#include <osgEarth/Feature>
#include <osgEarth/FeatureNode>
#include <osgEarth/Geometry>
#include <osgEarth/Style>
#include <osgEarth/LineSymbol>
#include <osgEarth/AltitudeSymbol>
#include <osgEarth/ExtrusionSymbol>
#include <osgEarth/PolygonSymbol>
#include <osgEarth/PointSymbol>
#include <osgEarth/GeoData>
#include <osgEarth/GeoTransform>
#include <osgEarth/ModelNode>

#include <osgViewer/Viewer>
#include <osgEarth/EarthManipulator>

namespace Ui {
class UdpReceiverWidget;
}

struct Track {
    int id;
    double longitude;
    double latitude;
    double altitude;
};

struct NavMessage {
  // Header
  char tag[3];           // 'N','A','V'
  uint8_t version;       // 0x01
  uint16_t total_len;    // expect 256
  uint8_t message_type;  // 0x03
  uint8_t flags;

  // Core
  uint32_t sequence;
  uint32_t epoch_seconds;
  uint32_t epoch_ms_residual;

  // Endpoints (IP in network order; port in host order)
  uint32_t source_ipv4_nbo;
  uint16_t source_port;
  uint32_t dest_ipv4_nbo;
  uint16_t dest_port;

  // Timestamp
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minutes;
  uint8_t seconds;
  uint16_t milliseconds;

  // Attitude, rates, heave
  float roll_deg;
  float pitch_deg;
  float heading_deg;
  float roll_rate_dps;
  float pitch_rate_dps;
  float heading_rate_dps;
  float heave_m;

  // Vel NED
  float vel_n_mps;
  float vel_e_mps;
  float vel_d_mps;

  // Geodetic
  double lat_deg;
  double lon_deg;
  float alt_m;

  // Std devs
  uint16_t lat_std;
  uint16_t lon_std;
  uint16_t vel_n_std;
  uint16_t vel_e_std;
  uint16_t vel_d_std;
  uint16_t roll_std;
  uint16_t pitch_std;
  uint16_t heading_std;

  // Altimeter and env
  float altimeter_depth_m;
  float altimeter_temp_c;
  uint16_t running_depth_mm;
  uint16_t reserved1;

  // Mission control
  uint8_t mission_cmd;
  uint8_t reserved2;

  float x_m;
  float y_m;
  float z_m;
  float surge_mps;
  float sway_mps;
};


class UdpReceiverWidget : public QWidget
{
    Q_OBJECT
public:
    explicit UdpReceiverWidget(AppMapView* rcMapView);
    ~UdpReceiverWidget();

    const QList<Track>& getTracks() const { return tracks; }

private slots:
    void onReadyRead();
    void clearData();

    void on_bindBtn_clicked(bool checked);

private:
    Ui::UdpReceiverWidget *ui;
    QUdpSocket* socket;
    QList<Track> tracks;
    bool isBound = false;
    AppMapView* m_mapView = nullptr;
    CGVectorPipe* m_udpvectorPipe = nullptr;
    QMap<int, QListWidgetItem*> m_idToListitem;
    QMap<int , CGFeatureObject> m_idToFeatureObject;
    CGPointer<CGFeatureLayer> m_udpTrackLayer;
    void setudpinitializeVectorLayer();
    bool insertOrUpdateUdpTrackonMap(int id, double latitude, double longitude, double altitude);
    void updateTotalPacketCount();
    ThreedPointHandler *m_3DPointHandler = nullptr;
    void insertMultipleTracksToLayer(const QList<Track>& tracks, CGFeatureObjectCollection& collection);
//    QList<Track> m_batchTracks;
//    int m_batchSize = 1000;
    int m_bulkInsertThreshold = 10;// default value
    QList<Track> m_trackBuffer;
//    RoutingWorker rt;
    void Update_location(double lat_deg, double lon_deg, double alt_m);
    void Add_location();
    bool parse_nav(const uint8_t* buf, size_t len, NavMessage& out, std::string& err);
    osgEarth::MapNode* m_mapNode = nullptr;
    osg::ref_ptr<osgEarth::LineString> _flightPath = nullptr;
    osg::ref_ptr<osgEarth::FeatureNode> _flightPathNode = nullptr;
    osg::ref_ptr<osg::Group> group = nullptr;

};
