#include "Simulation/mainwindow.h"
#include "Simulation/CommonSimParams.hpp"
#include "Simulation/SimObjects/Target.hpp"
#include "Simulation/SimObjects/Missile.hpp"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	setWindowIcon(QIcon(":/MGEIcon"));

	radiusCurve = new QCPCurve(ui->plot->xAxis, ui->plot->yAxis);
	QPen radCurvePen(QPen(QColor("green")));

	radCurvePen.setWidth(4);
	radiusCurve->setPen(radCurvePen);
	radiusCurve->setVisible(false);
	radiusCurve->setBrush(QBrush(QColor(0, 128, 0, 64)));
	radiusCurve->setName("Missile Proximity Radius");
	radiusCurve->setAdaptiveSampling(true);
	radiusCurve->setAntialiased(true);

	ui->plot->legend->setVisible(true);

	ui->plot->addGraph()->setName("Missile");
	ui->plot->addGraph()->setName("Target");
	
	ui->plot->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
	ui->plot->graph(0)->setLineStyle(QCPGraph::lsNone);
	ui->plot->graph(0)->setPen(QPen(QColor("red")));
	
	ui->plot->graph(1)->setScatterStyle(QCPScatterStyle::ssDisc);
	ui->plot->graph(1)->setLineStyle(QCPGraph::lsNone);
	ui->plot->graph(1)->setPen(QPen(QColor("blue")));
	
	ui->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom);

	std::thread dataPrepThread([&]{ prepareHitRadData(); });
	dataPrepThread.detach();
}

MainWindow::~MainWindow()
{
	delete ui;
	delete radiusCurve;
}

void MainWindow::on_startSimBtn_clicked()
{
	bool fileOutputNeeded = ui->fileOCheckBox->isChecked();
	double mslSpeed = ui->mslSpeedSpinBox->value();
	double tgtSpeed = ui->tgtSpeedSpinBox->value();
	double tgtYCoord = ui->distanceSpinBox->value();
	float tSinceReplot = 0;
	QPointF mslCoords(0, 0);
	QPointF tgtCoords(0, tgtYCoord);
	Simulation sim(tgtCoords, tgtSpeed, mslCoords, mslSpeed, fileOutputNeeded);

	ui->outputLabel->clear();
	ui->outputLabel->setText("Simulation's running; please wait");
	ui->outputLabel->setStyleSheet("QLabel { color : black; }");

	simFinished = false;
	mslX.clear(); mslY.clear();
	tgtX.clear(); tgtY.clear();
	tgtX.append(sim.getTarget()->getX());
	tgtY.append(sim.getTarget()->getY());
	mslX.append(sim.getMissile()->getX());
	mslY.append(sim.getMissile()->getY());
	plot();
	
	std::thread simThread([&]{ runSim(sim); });
	simThread.detach();

	while (!simFinished)
	{
		if (tSinceReplot >= 0.5)
		{
			tSinceReplot = 0;
			plot();
		}

		tSinceReplot += SIM_RESOLUTION;
	}

	if (sim.mslWithinTgtHitRadius())
	{
		ui->outputLabel->setText("Simulation's been stopped: the missile has reached the target");
		ui->outputLabel->setStyleSheet("QLabel { color : green; }");
	}
	else if (!sim.mslSpeedMoreThanTgtSpeed())
	{
		ui->outputLabel->setText("Simulation's been stopped: the missile's velocity has fallen below the target's");
		ui->outputLabel->setStyleSheet("QLabel { color : red; }");
	}

	plot(true, &sim);
}

void MainWindow::on_resetSimBtn_clicked()
{
	mslX.clear(); mslY.clear();
	tgtX.clear(); tgtY.clear();
	ui->outputLabel->clear();
	if (radiusCurve) radiusCurve->setVisible(false);
	simFinished = false;

	plot();
}

void MainWindow::plot(bool doFilter, Simulation* sim)
{
	if (doFilter)
	{
		auto filterData = [&](QVector<double>& keyVec, QVector<double>& valVec)
		{
			auto filterInterval = sim->getMissile()->getProxyRadius() * 2;
			
			if (keyVec.size() <= 1)
				return;

			for (auto i = 100; i < keyVec.size() - 100; ++i) // don't filter the some initial and the final points
			{
				auto pointDist = sqrt(pow(keyVec.at(i) - keyVec.at(i - 1), 2) + pow(valVec.at(i) - valVec.at(i - 1), 2));
				if (pointDist < filterInterval)
				{
					keyVec.remove(i);
					valVec.remove(i);
					i--;
				}
			}
		};

		filterData(mslX, mslY);
		filterData(tgtX, tgtY);
	}

	ui->plot->graph(0)->setData(mslX, mslY);
	ui->plot->graph(1)->setData(tgtX, tgtY);
	ui->plot->rescaleAxes();
	ui->plot->xAxis->setScaleRatio(ui->plot->yAxis, 1);
	ui->plot->yAxis->setScaleRatio(ui->plot->xAxis, 1);
	ui->plot->graph(0)->setAdaptiveSampling(true);
	ui->plot->graph(1)->setAdaptiveSampling(true);

	if (doFilter && sim)
	{
		auto mslFinalX = mslX.last();
		auto mslFinalY = mslY.last();
		QVector<double> xCoords, yCoords;
		
		xCoords.clear(); yCoords.clear();
		
		for (auto coordX : hitRadX)
			xCoords.append(coordX + mslFinalX);
		
		for (auto coordY : hitRadY)
			yCoords.append(coordY + mslFinalY);
		
		radiusCurve->setData(xCoords, yCoords);
	}

	ui->plot->replot();
	ui->plot->update();
}

void MainWindow::runSim(Simulation& sim)
{
	while (!simFinished)
	{
		sim.iterate();

		tgtX.append(sim.getTarget()->getX());
		tgtY.append(sim.getTarget()->getY());
		mslX.append(sim.getMissile()->getX());
		mslY.append(sim.getMissile()->getY());

		simFinished = sim.mslWithinTgtHitRadius() || !sim.mslSpeedMoreThanTgtSpeed();
	}
}

void MainWindow::prepareHitRadData()
{
	// TEMPORARY!
	const static double mslProxyRadius = 15;
	const static double mslProxyRadiusSq = std::pow(mslProxyRadius, 2);
	const static double coordStep { 0.5 };
	const static int stepCount = 4 * (mslProxyRadius / coordStep);
	double coordMult { 1 };

	hitRadX.append(mslProxyRadius);
	hitRadY.append(0);

	for (auto i = 0; i < stepCount; ++i)
	{
		auto lastX = hitRadX.last();
		
		if (lastX == -mslProxyRadius)
			coordMult *= -1;
		
		hitRadX.append(lastX - coordStep * coordMult);
		hitRadY.append(coordMult * std::sqrt(std::abs(mslProxyRadiusSq - std::pow(hitRadX.last(), 2))));
	}
}
