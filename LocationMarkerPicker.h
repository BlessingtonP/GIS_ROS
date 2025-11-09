#pragma once
#include <QObject>
#include <osgGA/GUIEventHandler>
#include <osgViewer/View>
#include <osgUtil/LineSegmentIntersector>

#include <osg/Geode>
#include <osg/ShapeDrawable>
#include <osg/BlendFunc>
#include <osg/Depth>
#include <osg/StateSet>

#include <osgEarth/MapNode>
#include <osgEarth/GeoData>
#include <osgEarth/SpatialReference>
#include <osgEarth/GeoTransform>

// Single-shot picker: arm() -> next left-click returns LLA via signal.
// Now also drops/updates a marker symbol on the map at the picked location.
class LocationMarkerPicker : public QObject, public osgGA::GUIEventHandler
{
    Q_OBJECT
public:
    explicit LocationMarkerPicker(osgEarth::MapNode* mapNode, QObject* parent=nullptr)
        : QObject(parent), _mapNode(mapNode) {}

public slots:
    void arm()                      { _armed = true; }
    void cancel()                   { _armed = false; emit pickCanceled(); }
    void setMarkerSizeMeters(double m) { _markerSizeM = (m > 0.0 ? m : _markerSizeM); }
    void setMarkerVisible(bool on)
    {
        _markerVisible = on;
        if (_markerXf.valid())
            _markerXf->setNodeMask(on ? ~0u : 0u);
    }
    void clearMarker()
    {
        if (_markerXf.valid())
        {
            // detach from all parents, then release
            osg::Node::ParentList parents = _markerXf->getParents();
            for (osg::Group* p : parents)
                if (p) p->removeChild(_markerXf.get());
            _markerXf = nullptr;
            _markerGeode = nullptr;
        }
    }

signals:
    void locationPicked(double latitude, double longitude, double altitude); // (lat, lon, alt)
    void pickFailed(const QString& reason);
    void pickCanceled();

public:
    bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa) override
    {
        if (!_armed) return false;
        if (ea.getEventType() != osgGA::GUIEventAdapter::PUSH ||
            ea.getButton()     != osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
            return false;

        auto* view = dynamic_cast<osgViewer::View*>(&aa);
        if (!view) return false;

        osgUtil::LineSegmentIntersector::Intersections hits;
        if (!view->computeIntersections(ea.getX(), ea.getY(), hits))
        {
            _armed = false;
            emit pickFailed(QStringLiteral("No intersection at clicked location"));
            return false;
        }

        // Use closest hit:
        const auto& hit = *hits.begin();
        osg::Vec3d world = hit.getWorldIntersectPoint();

        if (!_mapNode)
        {
            _armed = false;
            emit pickFailed(QStringLiteral("MapNode is null; cannot compute LLA"));
            return false;
        }

        // Convert WORLD -> map SRS -> geographic (WGS84) LLA:
        osgEarth::GeoPoint mapPt;
        mapPt.fromWorld(_mapNode->getMapSRS(), world); // world -> map SRS

        osgEarth::GeoPoint geo;
        const osgEarth::SpatialReference* geoSRS = _mapNode->getMapSRS()->getGeographicSRS();
        if (!mapPt.transform(geoSRS, geo))
        {
            _armed = false;
            emit pickFailed(QStringLiteral("Transform to geographic SRS failed"));
            return false;
        }

        // Place/update a marker node at this location (ABSOLUTE altitude to avoid clamp/toWorld issues)
        placeOrUpdateMarker(osgEarth::GeoPoint(_mapNode->getMapSRS(), geo.x(), geo.y(), geo.z(),
                                               osgEarth::ALTMODE_ABSOLUTE));

        _armed = false;

        // GeoPoint stores (x=lon, y=lat, z=alt). Emit (lat, lon, alt) per your requirement.
        emit locationPicked(geo.y(), geo.x(), geo.z());
        return true; // event handled
    }

private:
    // Build a simple red pin (cone + stem) in LOCAL space (meters)
    osg::Geode* createPinGeode(double sizeM) const
    {
        // sizeM roughly equals pin height
        const double stemH = 0.6 * sizeM;
        const double stemR = 0.06 * sizeM;
        const double coneH = 0.4 * sizeM;
        const double coneR = 0.16 * sizeM;

        osg::ref_ptr<osg::Geode> geode = new osg::Geode;

        // Stem (cylinder) centered along +Z
        osg::ref_ptr<osg::ShapeDrawable> stem =
            new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(0,0,stemH*0.5), stemR, stemH));
        stem->setColor(osg::Vec4(0.0f, 0.8f, 0.0f, 1.0f));    //0.90f, 0.05f, 0.1f, 1.0f - RED
        geode->addDrawable(stem.get());

        // Head (cone) sitting on top of stem
        osg::ref_ptr<osg::ShapeDrawable> cone =
            new osg::ShapeDrawable(new osg::Cone(osg::Vec3(0,0,stemH + coneH*0.5), coneR, coneH));
        cone->setColor(osg::Vec4(0.0f, 0.8f, 0.0f, 1.0f));    //0.90f, 0.05f, 0.1f, 1.0f  - RED
        geode->addDrawable(cone.get());

        // Basic state: depth test on, no lighting (use vertex color), no cull
        osg::StateSet* ss = geode->getOrCreateStateSet();
        ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
        ss->setMode(GL_CULL_FACE, osg::StateAttribute::OFF);
        ss->setMode(GL_DEPTH_TEST, osg::StateAttribute::ON);
        ss->setAttribute(new osg::Depth(osg::Depth::LESS));

        return geode.release();
    }

    void placeOrUpdateMarker(const osgEarth::GeoPoint& gpAbs)
    {
        if (!_markerVisible) return;

        if (!_markerXf.valid())
        {
            _markerXf = new osgEarth::GeoTransform;
            _markerXf->setTerrain(_mapNode->getTerrain()); // safe; alt mode is ABSOLUTE
            _markerGeode = createPinGeode(_markerSizeM);
            _markerXf->addChild(_markerGeode.get());
            _mapNode->addChild(_markerXf.get());
        }
        else
        {
            // Ensure only our geode is under the anchor
            _markerXf->removeChildren(0, _markerXf->getNumChildren());
            if (_markerGeode.valid())
                _markerXf->addChild(_markerGeode.get());
            else
                _markerXf->addChild(createPinGeode(_markerSizeM)); // fallback
        }

        _markerXf->setPosition(gpAbs);
        _markerXf->setNodeMask(_markerVisible ? ~0u : 0u);
    }

private:
    bool _armed = false;
    osg::observer_ptr<osgEarth::MapNode> _mapNode;

    // Marker state
    bool _markerVisible = true;
    double _markerSizeM = 15.0; // in meters (overall pin height)

    osg::ref_ptr<osgEarth::GeoTransform> _markerXf;
    osg::ref_ptr<osg::Geode>             _markerGeode;
};
