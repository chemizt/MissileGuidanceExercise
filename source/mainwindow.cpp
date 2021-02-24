#include "mainwindow.h"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    setWindowIcon(QIcon("./icon.ico"));
    ui->plot->legend->setVisible(true);
    ui->plot->addGraph()->setName("Ракета");
    ui->plot->addGraph()->setName("Цель");
    ui->plot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->plot->graph(0)->setLineStyle(QCPGraph::lsNone);
    ui->plot->graph(0)->setPen(QPen(QColor("red")));
    ui->plot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);
    ui->plot->graph(1)->setLineStyle(QCPGraph::lsNone);
    ui->plot->graph(1)->setPen(QPen(QColor("blue")));
    ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_startSimBtn_clicked()
{
    bool fileOutputNeeded = ui->fileOCheckBox->isChecked();
    bool simNotFinished = true;
    double mslSpeed = ui->mslSpeedSpinBox->value();
    double tgtSpeed = ui->tgtSpeedSpinBox->value();
    double tgtYCoord = ui->distanceSpinBox->value();
    QPointF mslCoords(0, 0);
    QPointF tgtCoords(0, tgtYCoord);
    Simulation sim(tgtCoords, tgtSpeed, mslCoords, mslSpeed, fileOutputNeeded);

    std::thread simThread([&]{ runSim(sim); });
    simThread.detach();

    ui->outputLabel->clear();
    mslX.clear(); mslY.clear();
    tgtX.clear(); tgtY.clear();
    plot();

    while (simNotFinished)
    {
        tgtX.append(sim.getTarget()->getX());
        tgtY.append(sim.getTarget()->getY());
        mslX.append(sim.getMissile()->getX());
        mslY.append(sim.getMissile()->getY());

        plot();

        simNotFinished = sim.mslNotWithinTgtHitRadius() && sim.mslSpeedMoreThanTgtSpeed();
    }
}

void MainWindow::on_resetSimBtn_clicked()
{
    mslX.clear(); mslY.clear();
    tgtX.clear(); tgtY.clear();
    plot();
}

void MainWindow::plot()
{
    ui->plot->graph(0)->setData(mslX, mslY);
    ui->plot->graph(1)->setData(tgtX, tgtY);
    ui->plot->rescaleAxes();
    ui->plot->xAxis->setScaleRatio(ui->plot->yAxis, 1);
    ui->plot->yAxis->setScaleRatio(ui->plot->xAxis, 1);
    ui->plot->replot();
    ui->plot->update();
}

void MainWindow::runSim(Simulation& sim)
{
    bool simNotFinished = true;

    while (simNotFinished)
    {
        sim.iterate();
        simNotFinished = sim.mslNotWithinTgtHitRadius() && sim.mslSpeedMoreThanTgtSpeed();
    }

    if (!sim.mslNotWithinTgtHitRadius())
    {
        ui->outputLabel->setText("Симуляция завершена: ракета поразила цель");
        ui->outputLabel->setStyleSheet("QLabel { color : green; }");
    }
    else if (!sim.mslSpeedMoreThanTgtSpeed())
    {
        ui->outputLabel->setText("Симуляция завершена: скорость ракеты упала ниже скорости цели");
        ui->outputLabel->setStyleSheet("QLabel { color : red; }");
    }
}
