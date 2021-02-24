#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <thread>

#include "simulation.hpp"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

    public:
        MainWindow(QWidget *parent = nullptr);
        ~MainWindow();


    private slots:
        void on_startSimBtn_clicked();
        void on_resetSimBtn_clicked();

private:
        Ui::MainWindow *ui;
        QVector<double> mslX, mslY, tgtX, tgtY;
        void plot();
        void runSim(Simulation& sim);

};
#endif // MAINWINDOW_H
