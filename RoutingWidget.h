#ifndef ROUTINGWIDGET_H
#define ROUTINGWIDGET_H

#include <QWidget>

#include <ComponentDialogs/locationdialog.h>

#include <Tools/loadingwidget.h>
#include <osgEarth/MapNode>

QT_BEGIN_NAMESPACE
namespace Ui { class RoutingWidget; }
QT_END_NAMESPACE

class RoutingWidget : public QWidget
{
    Q_OBJECT
public:
    explicit RoutingWidget(AppMapView *rcMapView, osgEarth::MapNode* mapNode = nullptr);
    ~RoutingWidget();
private slots:
    void selectShapefile();
    void computePath();
    void resetFields();
    void appendLog(const QString &msg);
    void onRoutingFinished(const QString &outPath);
private:
    Ui::RoutingWidget *ui;
    QString shapefilePath;
    void validateShapefile(const QString &path);
    void runRoutingInThread(double sx, double sy, double ex, double ey);

    LocationDialog *m_txLocationDialog;

    LocationDialog *m_rxLocationDialog;

    CGPointF m_TxLoc,m_RxLoc;

    AppMapView *m_mapView;

    QString prevOP;

    LoadingWidget *m_loadingWidget = nullptr;
    
    osgEarth::MapNode* m_mapNode = nullptr;

};

#endif // ROUTINGWIDGET_H
