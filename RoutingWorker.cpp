#include "RoutingWorker.h"
#include <ogrsf_frmts.h>
#include <QFileInfo>
#include <QDateTime>
#include <QFile>
#include <QDataStream>
#include <QTextStream>
#include <QDir>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <QtMath>               // for qDegreesToRadians, qSin, qCos, etc.
#include <queue>
#include <unordered_map>
#include <vector>
#include <limits>
#include <QDebug>

#include <QUdpSocket>
#include <QHostAddress>
#include <QTimer>
#include <QDebug>

// Haversine‐based geodetic distance (meters)
static double geodeticDistance(double lon1, double lat1, double lon2, double lat2) {
    static const double R = 6371000.0; // earth radius in meters
    double dLat = qDegreesToRadians(lat2 - lat1);
    double dLon = qDegreesToRadians(lon2 - lon1);
    double a = qSin(dLat/2) * qSin(dLat/2)
            + qCos(qDegreesToRadians(lat1)) * qCos(qDegreesToRadians(lat2))
            * qSin(dLon/2) * qSin(dLon/2);
    double c = 2 * qAtan2(qSqrt(a), qSqrt(1 - a));
    return R * c;
}

// Hash for PtKey
namespace std {
template<> struct hash<RoutingWorker::PtKey> {
    size_t operator()(RoutingWorker::PtKey const& p) const noexcept {
        auto h1 = std::hash<long long>()(std::llround(p.x*1e6));
        auto h2 = std::hash<long long>()(std::llround(p.y*1e6));
        return h1 ^ (h2<<1);
    }
};
}

RoutingWorker::RoutingWorker(const QString &shapefile,
                             double sx,double sy,
                             double ex,double ey,
                             AlgorithmType algo,
                             osgEarth::MapNode* mapNode)
    : inputPath(shapefile)
    , startX(sx), startY(sy)
    , endX(ex),   endY(ey)
    , algoType(algo)
    ,m_mapNode(mapNode)
{

    Add_location();

    //Connect_UDP();

}

RoutingWorker::RoutingWorker()
{

}



void RoutingWorker::process()
{
    emit stageUpdate("Checking graph cache...");
    QFileInfo fi(inputPath);
    QString cachePath = fi.absolutePath() + "/" + fi.completeBaseName() + ".graphcache";
    qint64 srcTs = fi.lastModified().toMSecsSinceEpoch();

    std::vector<PtKey> nodes;
    std::vector<std::vector<Edge>> adj;

    if (QFile::exists(cachePath)) {
        QFile cache(cachePath);
        if (cache.open(QIODevice::ReadOnly)) {
            QDataStream in(&cache);
            qint64 savedTs = 0;
            in >> savedTs;
            if (savedTs == srcTs) {
                emit stageUpdate("Loading graph from cache...");
                int nNodes = 0;
                in >> nNodes;
                nodes.resize(nNodes);
                for (int i = 0; i < nNodes; ++i) in >> nodes[i].x >> nodes[i].y;
                adj.resize(nNodes);
                for (int u = 0; u < nNodes; ++u) {
                    int deg = 0; in >> deg;
                    adj[u].resize(deg);
                    for (int j = 0; j < deg; ++j) in >> adj[u][j].to >> adj[u][j].cost;
                }
                cache.close();
            } else {
                cache.close();
                QFile::remove(cachePath);
                emit stageUpdate("Cache outdated; rebuilding graph...");
                goto BUILD_GRAPH;
            }
        } else {
            emit stageUpdate("Cannot open cache; rebuilding graph...");
            goto BUILD_GRAPH;
        }
    } else {
BUILD_GRAPH:
        emit stageUpdate("Registering GDAL drivers...");
        GDALAllRegister();

        emit stageUpdate("Opening vector dataset...");
        GDALDataset *inDS = static_cast<GDALDataset*>(
                    GDALOpenEx(inputPath.toStdString().c_str(), GDAL_OF_VECTOR, nullptr, nullptr, nullptr)
                    );
        if (!inDS) {
            emit stageUpdate("ERROR: cannot open input vector file.");
            emit finished();
            return;
        }

        OGRLayer *layer = inDS->GetLayer(0);
        if (!layer) {
            emit stageUpdate("ERROR: dataset has no layers.");
            GDALClose(inDS);
            emit finished();
            return;
        }

        // --- FIX: recompute and validate extent ---
        OGREnvelope env;
        if (layer->GetExtent(&env, /*force=*/true) != OGRERR_NONE ||
                !(env.MinX < env.MaxX && env.MinY < env.MaxY))
        {
            emit stageUpdate("Error: Border layer extent is invalid (empty or non-finite).");
            GDALClose(inDS);
            emit finished();
            return;
        }
        emit stageUpdate(QString("Layer extent: [%1, %2] x [%3, %4]")
                         .arg(env.MinX, 0, 'f', 6).arg(env.MaxX, 0, 'f', 6)
                         .arg(env.MinY, 0, 'f', 6).arg(env.MaxY, 0, 'f', 6));

        // Report declared geometry (may be Unknown)
        auto typeToStr = [](OGRwkbGeometryType t)->QString {
            switch (wkbFlatten(t)) {
            case wkbPoint: return "Point";
            case wkbLineString: return "LineString";
            case wkbPolygon: return "Polygon";
            case wkbMultiLineString: return "MultiLineString";
            case wkbMultiPolygon: return "MultiPolygon";
            case wkbMultiPoint: return "MultiPoint";
            case wkbGeometryCollection: return "GeometryCollection";
            case wkbUnknown: default: return "Unknown";
            }
        };
        OGRwkbGeometryType declared = wkbFlatten(layer->GetGeomType());
        emit stageUpdate(QString("Layer declared geometry: \"%1\"").arg(typeToStr(declared)));

        // --- FIX: accept Unknown/MultiLineString by inspecting features ---
        if (declared != wkbUnknown &&
                declared != wkbLineString &&
                declared != wkbMultiLineString)
        {
            emit stageUpdate("ERROR: Expected LineString/MultiLineString geometries.");
            GDALClose(inDS);
            emit finished();
            return;
        }

        emit stageUpdate("Building graph from features...");
        std::unordered_map<PtKey,int> nodeMap;
        nodes.clear(); adj.clear();

        auto addOrGet = [&](const PtKey &p)->int {
            auto it = nodeMap.find(p);
            if (it != nodeMap.end()) return it->second;
            int id = static_cast<int>(nodes.size());
            nodeMap.emplace(p, id);
            nodes.push_back(p);
            adj.emplace_back();
            return id;
        };

        auto addLineString = [&](OGRLineString *ls) {
            if (!ls) return;
            const int nPt = ls->getNumPoints();
            if (nPt < 2) return; // skip degenerate
            for (int i = 0; i + 1 < nPt; ++i) {
                PtKey a{ ls->getX(i),   ls->getY(i)   };
                PtKey b{ ls->getX(i+1), ls->getY(i+1) };
                if (!std::isfinite(a.x) || !std::isfinite(a.y) ||
                        !std::isfinite(b.x) || !std::isfinite(b.y)) continue;
                int ia = addOrGet(a);
                int ib = addOrGet(b);
                double cost = euclid(a.x, a.y, b.x, b.y);
                adj[ia].push_back({ ib, cost });
                adj[ib].push_back({ ia, cost });
            }
        };

        layer->ResetReading();
        long long featCount = 0;
        for (OGRFeature *feat = nullptr; (feat = layer->GetNextFeature()) != nullptr; ) {
            ++featCount;
            OGRGeometry *g = feat->GetGeometryRef();
            if (g) {
                OGRwkbGeometryType gt = wkbFlatten(g->getGeometryType());
                if (gt == wkbLineString) {
                    addLineString(g->toLineString());
                } else if (gt == wkbMultiLineString) {
                    OGRMultiLineString *mls = g->toMultiLineString();
                    if (mls) {
                        const int n = mls->getNumGeometries();
                        for (int i = 0; i < n; ++i) {
                            OGRGeometry *part = mls->getGeometryRef(i);
                            if (part) addLineString(part->toLineString());
                        }
                    }
                } // ignore other geom types silently
            }
            OGRFeature::DestroyFeature(feat);
        }

        if (nodes.empty()) {
            emit stageUpdate("ERROR: No usable LineString/MultiLineString features found.");
            GDALClose(inDS);
            emit finished();
            return;
        }

        GDALClose(inDS);
        emit stageUpdate(QString("Graph built: %1 nodes, from %2 features")
                         .arg(nodes.size()).arg(featCount));

        // Save to cache
        QFile cache(cachePath);
        if (cache.open(QIODevice::WriteOnly)) {
            emit stageUpdate("Saving graph cache...");
            QDataStream out(&cache);
            out << srcTs;
            int nNodes = static_cast<int>(nodes.size());
            out << nNodes;
            for (auto &n : nodes) out << n.x << n.y;
            for (int u = 0; u < nNodes; ++u) {
                int deg = static_cast<int>(adj[u].size());
                out << deg;
                for (auto &e : adj[u]) out << e.to << e.cost;
            }
            cache.close();
        } else {
            emit stageUpdate("WARNING: failed to write cache.");
        }
    }

    // --- Snap start/end safely ---
    emit stageUpdate("Snapping start/end...");
    if (nodes.empty()) { emit stageUpdate("ERROR: Graph is empty."); emit finished(); return; }
    int sN = 0, eN = 0;
    double bestS = std::numeric_limits<double>::infinity();
    double bestE = std::numeric_limits<double>::infinity();
    for (int i = 0; i < static_cast<int>(nodes.size()); ++i) {
        double ds = euclid(startX, startY, nodes[i].x, nodes[i].y);
        if (ds < bestS) { bestS = ds; sN = i; }
        double de = euclid(endX, endY, nodes[i].x, nodes[i].y);
        if (de < bestE) { bestE = de; eN = i; }
    }
    emit stageUpdate(QString("Start → node %1 at (%2, %3)").arg(sN).arg(nodes[sN].x).arg(nodes[sN].y));
    emit stageUpdate(QString("End   → node %1 at (%2, %3)").arg(eN).arg(nodes[eN].x).arg(nodes[eN].y));

    // --- Connectivity check ---
    emit stageUpdate("Checking connectivity...");
    std::vector<bool> seen(nodes.size(), false);
    std::queue<int> qq;
    seen[sN] = true; qq.push(sN);
    while (!qq.empty()) {
        int u = qq.front(); qq.pop();
        for (auto &ed : adj[u]) if (!seen[ed.to]) { seen[ed.to] = true; qq.push(ed.to); }
    }
    if (!seen[eN]) {
        emit stageUpdate("Graph NOT connected → abort");
        emit finished();
        emit routingComplete("Graph NOT connected → abort");
        return;
    }
    emit stageUpdate("Graph connected, proceeding...");

    // --- Shortest path ---
    std::vector<int> prev(nodes.size(), -1), path;
    bool pathFound = false;
    QString suffix;

    if (algoType == Dijkstra) {
        emit stageUpdate("Running Dijkstra...");
        std::vector<double> dist(nodes.size(), std::numeric_limits<double>::infinity());
        using P = std::pair<double,int>;
        std::priority_queue<P, std::vector<P>, std::greater<P>> pq;
        dist[sN] = 0; pq.push({0, sN});
        while (!pq.empty()) {
            auto [d,u] = pq.top(); pq.pop();
                    if (d > dist[u]) continue;
                    if (u == eN) break;
                    for (auto &ed : adj[u]) {
                double nd = d + ed.cost;
                if (nd < dist[ed.to]) {
                    dist[ed.to] = nd;
                    prev[ed.to] = u;
                    pq.push({nd, ed.to});
                }
            }
        }
        for (int at = eN; at != -1; at = prev[at]) path.push_back(at);
        if (!path.empty() && path.back() == sN) {
            std::reverse(path.begin(), path.end());
            emit stageUpdate(QString("Dijkstra: %1 nodes").arg(path.size()));
            pathFound = true; suffix = "_dijkstra";
        } else {
            emit stageUpdate("Dijkstra: no path");
        }
    } else {
        emit stageUpdate("Running A*...");
        auto h = [&](int idx){ return euclid(nodes[idx].x, nodes[idx].y, endX, endY); };
        struct Node { int idx; double f; };
        struct Cmp { bool operator()(Node const&a, Node const&b) const { return a.f > b.f; } };
        std::vector<double> g(nodes.size(), std::numeric_limits<double>::infinity());
        std::priority_queue<Node, std::vector<Node>, Cmp> pq;
        g[sN] = 0; pq.push({sN, h(sN)});
        while (!pq.empty()) {
            auto cur = pq.top(); pq.pop();
            int u = cur.idx;
            if (u == eN) break;
            if (cur.f - h(u) > g[u]) continue;
            for (auto &ed : adj[u]) {
                double tg = g[u] + ed.cost;
                if (tg < g[ed.to]) {
                    g[ed.to] = tg;
                    prev[ed.to] = u;
                    pq.push({ed.to, tg + h(ed.to)});
                }
            }
        }
        for (int at = eN; at != -1; at = prev[at]) path.push_back(at);
        if (!path.empty() && path.back() == sN) {
            std::reverse(path.begin(), path.end());
            emit stageUpdate(QString("A*: %1 nodes").arg(path.size()));
            pathFound = true; suffix = "_astar";
        } else {
            emit stageUpdate("A*: no path");
            emit routingComplete("Error. No connectivity between start and end points.");
        }
    }

    if (pathFound) {
        // Geodetic length (assumes lon/lat degrees; adjust if projected)
        double totalMeters = 0.0;
        for (int i = 1; i < static_cast<int>(path.size()); ++i) {
            const PtKey &A = nodes[path[i-1]];
            const PtKey &B = nodes[path[i]];
            totalMeters += geodeticDistance(A.x, A.y, B.x, B.y);
        }
        emit stageUpdate(QString("\nPath length: %1 m (%2 km)")
                         .arg(totalMeters, 0, 'f', 2)
                         .arg(totalMeters/1000.0, 0, 'f', 3));

        // Ensure output dir
        QDir curr = QDir::current();
        if (!curr.exists("RouteFinderOutput")) curr.mkdir("RouteFinderOutput");
        QString outDir = curr.filePath("RouteFinderOutput");
        QString timestamp = QDateTime::currentDateTime().toString("hhmmss");
        QString base = outDir + QDir::separator() + "route" + suffix + timestamp;
        QString shpPath = base + ".shp";
        QString csvPath = base + ".csv";

        // write shapefile with proper CRS and extent
        emit stageUpdate("Writing SHP to " + shpPath + "...");

        // Register drivers
        GDALAllRegister();

        // Get ESRI Shapefile driver
        GDALDriver *drv = GetGDALDriverManager()->GetDriverByName("ESRI Shapefile");
        if (!drv) {
            emit stageUpdate("ERROR: ESRI Shapefile driver not available");
            emit finished();
            return;
        }

        // Remove existing file if it exists
        if (QFile::exists(shpPath)) {
            QFile::remove(shpPath);
        }

        // Create dataset
        GDALDataset *outDS = drv->Create(shpPath.toStdString().c_str(), 0, 0, 0, GDT_Unknown, nullptr);
        if (!outDS) {
            emit stageUpdate("ERROR: Cannot create shapefile");
            emit finished();
            return;
        }

        // Create spatial reference (WGS84)
        OGRSpatialReference srs;
        srs.SetWellKnownGeogCS("WGS84");

        // Create layer with LineString type and WGS84 CRS
        OGRLayer *lay = outDS->CreateLayer("route", &srs, wkbLineString, nullptr);
        if (!lay) {
            emit stageUpdate("ERROR: Cannot create layer");
            GDALClose(outDS);
            emit finished();
            return;
        }

        // Create line geometry
        OGRLineString line;
        for (int idx: path) {
            line.addPoint(nodes[idx].x, nodes[idx].y);
            qDebug() << "Latitude: INSIDE LOOP" << nodes[idx].x <<  "Longitude: " << nodes[idx].y;
        }

        //============================================================

//        try {
//        UdpSocket sock;

//        if (!sock.try_connect_remote("192.168.2.10", 4500)) {
//        //if (!sock.try_connect_remote("0.0.0.0", 4500)) {
//           std::cerr << "[Error] Failed to connect to remote 192.168.1.50:4500\n";
//           //return 1;
//        }
//        else
//        {
//            qDebug("TRY CONNECT IS SUCCEDED");
//        }

//        if (!sock.is_ready()) {
//           std::cerr << "[Error] Socket not ready\n";
//           //return 1;
//        }
//        else {
//            qDebug("IS READY IS PASSED");
//        }

//        MessageSender<Route, RteSerializer> sender(std::move(sock),
//                                                       RteSerializer{});

//        Route route;
//        //qDebug() << "++++size of route structure: " << sizeof(route);

//        route.sequence = 1;		//This will increment
//        route.epoch_seconds = 1690000000;	//This is the local eppoch time
//        route.epoch_millis_residual = 123;		//milli sec epoch residual
//        route.route_id = 42;	//Some random const

//        auto [src_ip, src_port] = sender.socket().local_address();
//        auto [dst_ip, dst_port] = sender.socket().remote_address();

//        route.source_ipv4 =
//                ::inet_addr("192.168.2.11");  // returns network byte order
//        route.source_port = 4500;
//        route.dest_ipv4 = ::inet_addr("192.168.2.10");
//        route.dest_port = 4500;

//        uint16_t serial_no = 1;
//        uint16_t running_depth_m = 20;
//        uint8_t speed_mps = 3;

//        qDebug() << "++++Total num points in the Line: " << line.getNumPoints();

//        for (int i = 0; i < line.getNumPoints(); i++) {
//            double lon = line.getX(i); // X = longitude
//            double lat = line.getY(i); // Y = latitude

//            qDebug() << "Latitude: INSIDE LOOP" << lon <<  "Longitude: " << lat;

//            Waypoint wp;
//            //qDebug() << "++++size of wp structure: " << sizeof(wp);
//            wp = make_waypoint(
//                serial_no++,      // serial number increments
//                lat, lon,         // latitude, longitude
//                running_depth_m++,               // example depth
//                speed_mps++                 // example speed
//            );
//            route.waypoints.push_back(wp);
//            }

//        // Serialize the message first to compute size
//        RteSerializer serializer;
//        auto serialized_data = serializer.serialize(route);
//        qDebug() << "+++++Serialized Route message size:" << serialized_data.size() << "bytes";

//            sender.send_message(route);
//            qDebug() << "+++++Sent Route message with " << route.waypoints.size()
//                      << " waypoints to GIS client.\n";
//        } catch (const std::exception& e) {
//            std::cerr << "[Error] " << e.what() << "\n";
//            //return 1;
//        }

        QUdpSocket udpSocket;
        QHostAddress localAddress("192.168.2.12");
        QHostAddress remoteAddress("192.168.2.13");
        quint16 localPort = 4501;
        quint16 remotePort = 4500;

//        if (!udpSocket.bind(localAddress, localPort)) {
//            qCritical() << "[Error] Failed to bind to local address.";
//            return;
//        }

        qDebug() << "[Info] Bound to" << localAddress.toString() << ":" << localPort;

        // --- Prepare Route message ---
        Route route;
        route.sequence = 1;
        route.epoch_seconds = 1690000000;
        route.epoch_millis_residual = 123;
        route.route_id = 42;

        route.source_ipv4 = localAddress.toIPv4Address();
        route.source_port = localPort;
        route.dest_ipv4 = remoteAddress.toIPv4Address();
        route.dest_port = remotePort;

        uint16_t serial_no = 1;
        uint16_t running_depth_m = 20;
        uint8_t speed_mps = 3;

//        qDebug() << "++++Total num points in the Line:" << line.getNumPoints();

        for (int i = 0; i < line.getNumPoints(); ++i) {
            double lon = line.getX(i);
            double lat = line.getY(i);

            qDebug() << "Waypoint #" << serial_no << "Lat:" << lat << "Lon:" << lon;

            Waypoint wp = make_waypoint(
                serial_no++,
                lat, lon,
                running_depth_m++,
                speed_mps++
            );

            route.waypoints.push_back(wp);
        }
        
    // --------------------------
    // Step 2: Compute fragmentation
    // --------------------------
    const size_t totalWaypoints = route.waypoints.size();
    const size_t maxPerFragment = 63;
    const uint8_t totalFragments =
        static_cast<uint8_t>((totalWaypoints + maxPerFragment - 1) / maxPerFragment);

    qDebug() << "[Fragmentation] Total waypoints:" << totalWaypoints
             << "→ Fragments:" << (int)totalFragments;

    // --------------------------
    // Step 3: Send fragments
    // --------------------------
    for (uint8_t fragIdx = 0; fragIdx < totalFragments; ++fragIdx)
    {
        size_t start = fragIdx * maxPerFragment;
        size_t end = std::min(start + maxPerFragment, totalWaypoints);
        size_t count = end - start;

        Route frag = route; // Copy base route
        frag.Fragment_index = fragIdx;
        frag.Fragment_count = totalFragments;

        frag.waypoints.assign(
            route.waypoints.begin() + start,
            route.waypoints.begin() + end);

        qDebug().noquote()
            << "\n[DEBUG] Preparing fragment:"
            << fragIdx + 1 << "/" << totalFragments
            << "\n  Start index     :" << start
            << "\n  End index       :" << end
            << "\n  Waypoints count :" << count
            << "\n  Fragment_index  :" << frag.Fragment_index
            << "\n  Fragment_count  :" << frag.Fragment_count;

        // Serialize this fragment
        RteSerializer serializer;
        std::vector<uint8_t> fragData = serializer.serialize(frag);
        QByteArray datagram(reinterpret_cast<const char*>(fragData.data()),
                            static_cast<int>(fragData.size()));

        qint64 bytesSent = udpSocket.writeDatagram(datagram, remoteAddress, remotePort);

        if (bytesSent == -1)
        {
            qCritical() << "[Error] Failed to send fragment"
                        << fragIdx << ":" << udpSocket.errorString();
        }
        else
        {
            qDebug() << "[Sent] Fragment"
                     << fragIdx + 1 << "/"
                     << totalFragments
                     << "→ waypoints:" << count
                     << "bytes:" << bytesSent
                     << "| Fragment_index:" << frag.Fragment_index
                     << "| Fragment_count:" << frag.Fragment_count;
        }

//        QThread::msleep(50); // small delay between fragments
    }        

        // --- Serialize ---
//        RteSerializer serializer;
//        std::vector<uint8_t> data = serializer.serialize(route);
//        QByteArray datagram(reinterpret_cast<const char*>(data.data()), static_cast<int>(data.size()));

 //       qDebug() << "+++++Serialized Route message size:" << datagram.size() << "bytes";

        // --- Send ---
//        qint64 bytesSent = udpSocket.writeDatagram(datagram, remoteAddress, remotePort);

//        if (bytesSent == -1) {
//            qCritical() << "[Error] Failed to send datagram:" << udpSocket.errorString();
//        } else {
//           qDebug() << "+++++Sent Route message with"
//                     << route.waypoints.size()
//                     << "waypoints to GIS client.";
//        }

        // Quit after short delay to allow buffer to flush
       // QTimer::singleShot(100, &app, &QCoreApplication::quit);


        //===========================================================

        // Ensure the line has at least 2 points
        if (line.getNumPoints() < 2) {
            emit stageUpdate("ERROR: LineString must have at least 2 points");
            GDALClose(outDS);
            emit finished();
            return;
        }

 //       qDebug() << "Total num points in the Line: " << line.getNumPoints();

        // Create feature and set geometry
        OGRFeature *of = OGRFeature::CreateFeature(lay->GetLayerDefn());
        of->SetGeometry(&line);

        // Create the feature in the layer
        if (lay->CreateFeature(of) != OGRERR_NONE) {
            emit stageUpdate("ERROR: Failed to create feature");
            OGRFeature::DestroyFeature(of);
            GDALClose(outDS);
            emit finished();
            return;
        }

        // Clean up
        OGRFeature::DestroyFeature(of);
        GDALClose(outDS);

        // Create the .prj file for WGS84
        QString prjPath = base + ".prj";
        QFile prjFile(prjPath);
        if (prjFile.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream prjStream(&prjFile);
            prjStream << "GEOGCS[\"WGS 84\",DATUM[\"WGS_1984\",SPHEROID[\"WGS 84\",6378137,298.257223563,AUTHORITY[\"EPSG\",\"7030\"]],AUTHORITY[\"EPSG\",\"6326\"]],PRIMEM[\"Greenwich\",0,AUTHORITY[\"EPSG\",\"8901\"]],UNIT[\"degree\",0.0174532925199433,AUTHORITY[\"EPSG\",\"9122\"]],AXIS[\"Latitude\",NORTH],AXIS[\"Longitude\",EAST],AUTHORITY[\"EPSG\",\"4326\"]]";
            prjFile.close();
        }

        emit stageUpdate("SHP written with WGS84 CRS.");
        // CSV
        emit stageUpdate("Writing CSV to " + csvPath + "...");
        QFile csv(csvPath);
        if (csv.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QTextStream ts(&csv);
            ts << "lon,lat\n";
            for (int idx : path)
                ts << QString::number(nodes[idx].x,'f',6) << ","
                   << QString::number(nodes[idx].y,'f',6) << "\n";
            csv.close();
            emit stageUpdate("CSV written.");
        } else {
            emit stageUpdate("ERROR: cannot write CSV");
        }

        emit routingComplete(shpPath + ";" + csvPath);
    }

    //Connect_UDP();

    emit finished();
}

void RoutingWorker::Connect_UDP()
{

    qDebug("CAME INSIDE CONNECT UDP FUNCTION");

    const char* remote_ip = "192.168.2.10";
    uint16_t remote_port = 4500;

    UdpSocket sock;

    // You must bind to receive UDP data
    const char* bind_ip = "192.168.2.11";  // Local IP (or use "0.0.0.0" to listen on all)
    uint16_t bind_port = 4500;

    if (!sock.try_bind(bind_ip, bind_port)) {
        perror("bind failed");
        return;
    } else {
        qDebug("++++ Able to bind local IP and port");
    }

    // 1. Send "hello" message to the server
//    const char* hello_msg = "hello";
//    sock.send_to(reinterpret_cast<const uint8_t*>(hello_msg),
//                                strlen(hello_msg),
//                                server_ip,
//                                server_port);

//    qDebug() << "++++Sent 'hello' to server at" << server_ip << ":" << server_port;

    std::vector<uint8_t> buf(2048);

    qDebug() << "Bound socket fd =" << sock.fd_;

    for (;;) {
      uint32_t src_ip_nbo = 0;
      uint16_t src_port = 0;
      ssize_t n = sock.recv_from(buf.data(), buf.size(), &src_ip_nbo, &src_port);
      if (n < 0) {
          if (errno == EINTR) continue;

          int errnum = errno;
          qDebug() << "recvfrom failed, errno =" << errnum << "(" << strerror(errnum) << ")";
          return;
      }
      else
      {
          qDebug("No Data Received");
      }

      if (n < 256) {
        std::cerr << "Short datagram (" << n << " bytes), expected >= 256\n";
        continue;
      }

      NavMessage msg{};
      std::string err;
      if (!parse_nav(buf.data(), static_cast<size_t>(n), msg, err)) {
        std::cerr << "Parse_nav failed: " << err << "\n";
        continue;
      }

      if (n > 256) {
        std::cerr << "Warning: datagram length " << n << " > 256, ignoring trailing bytes\n";
      }

      std::cout << "NAV seq=" << msg.sequence << " from "
                << ipv4_to_string(src_ip_nbo) << ":" << src_port << " pos=("
                << msg.lat_deg << "," << msg.lon_deg << ") alt=" << msg.alt_m
                << " hdg=" << msg.heading_deg << " velN/E/D=(" << msg.vel_n_mps
                << "," << msg.vel_e_mps << "," << msg.vel_d_mps << ")"
                << "\n";
    }
}


bool RoutingWorker::parse_nav(const uint8_t* buf, size_t len, NavMessage& out, std::string& err)
{
    qDebug("Came INTO Parse_nav function");

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

std::string RoutingWorker::ipv4_to_string(uint32_t ipv4_nbo)
{
  in_addr a{.s_addr = ipv4_nbo};
  char buf[INET_ADDRSTRLEN] = {0};
  const char* s = ::inet_ntop(AF_INET, &a, buf, sizeof(buf));
  return s ? std::string(s) : std::string();
}

void RoutingWorker::Add_location()
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


void RoutingWorker::Update_location(double lat_deg, double lon_deg, double alt_m)
{
    std::cout << "[DEBUG] Entered INTO Update_location()" << std::endl;
    std::cout << "[DEBUG] lat = " << lat_deg << ", lon = " << lon_deg << ", alt = " << alt_m << std::endl;

    std::cout << "[DEBUG] BEFORE GeoPOINT Update_location()" << std::endl;

    if (!m_mapNode)
    {
        std::cout << "[FATAL] m_mapNode is nullptr!" << std::endl;
        return;
    }
    else
    {
        std::cout << "[DEBUG] m_mapNode is valid." << std::endl;
    }

    osgEarth::GeoPoint point(m_mapNode->getMapSRS(), lon_deg , lat_deg, alt_m, osgEarth::ALTMODE_RELATIVE);

    std::cout << "[DEBUG] AFTER GeoPOINT Update_location()" << std::endl;

    // Append to path
    if (_flightPath.valid())
    {
        _flightPath->push_back(point.vec3d());
        std::cout << "[DEBUG] Point added to _flightPath: " << point.vec3d().x() << ", "
                  << point.vec3d().y() << ", " << point.vec3d().z() << std::endl;
    }
    else
    {
        std::cout << "[WARN] _flightPath is not valid!" << std::endl;
    }

    std::cout << "[DEBUG] Total points in _flightPath: " << _flightPath->size() << std::endl;

    if (!_flightPathNode.valid())
    {
        std::cout << "[WARN] _flightPathNode is not valid!" << std::endl;
        return;
    }
    else
    {
        std::cout << "[DEBUG] _flightPathNode is valid!" << std::endl;
    }

    auto feature1 = _flightPathNode->getFeature();
    if (!feature1)
    {
        std::cout << "[WARN] _flightPathNode->getFeature() returned nullptr!" << std::endl;
        return;
    }
    if (feature1)
    {
        feature1->setGeometry(_flightPath);
        _flightPathNode->dirty();
        std::cout << "[DEBUG] Feature geometry updated and _flightPathNode marked dirty." << std::endl;
    }
    else
    {
        std::cout << "[WARN] _flightPathNode has no valid feature!" << std::endl;
    }
}

