#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <QMainWindow>
#include <QGraphicsScene>
#include <QGraphicsEllipseItem>
#include <QGraphicsRectItem>
#include <QGraphicsPixmapItem>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <boost/filesystem/path.hpp>
#include "QtRosNode.h"
#include "YamlParser.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    QtRosNode* qtRosNode;
    YamlParser* yamlParser;

    void setRosNode(QtRosNode* qtRosNode);
    void setYamlParser(YamlParser* yamlParser);
    void closeEvent(QCloseEvent *event);

public slots:
    //Slots for signals emitted in the QtRosNode (e.g. a topic is received)
    void updateGraphicsReceived();
    void btnFwdPressed();
    void btnFwdReleased();
    void btnBwdPressed();    
    void btnBwdReleased();
    void btnLeftPressed();
    void btnLeftReleased();
    void btnRightPressed();
    void btnRightReleased();
    void btnCmdVelPressed();
    void btnCmdVelReleased();

    void navBtnCalcPath_pressed();
    void navBtnExecPath_pressed();

    void torSbPosValueChanged(double d);
    void laSbAnglesValueChanged(double d);
    void raSbAnglesValueChanged(double d);
    void laTxtPredefinedReturnPressed();
    void raTxtPredefinedReturnPressed();
    void laSbGripperValueChanged(double d);
    void raSbGripperValueChanged(double d);
    void hdSbHeadValueChanged(double d);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
