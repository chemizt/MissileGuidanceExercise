#include "Simulation/mainwindow.h"
#include "Simulation/CommonSimParams.hpp"
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
	ui->setupUi(this);
	// setWindowIcon(QIcon(":/MGEIcon"));
	setWindowIcon(QIcon(":/icon.ico"));
	_leSim = new Simulation();

	radiusCurve = new QCPCurve(ui->plot->xAxis, ui->plot->yAxis);
	auto radCrvCasted = static_cast<QCPCurve*>(radiusCurve);
	QPen radCurvePen(QPen(QColor("green")));

	radCurvePen.setWidth(4);
	radCrvCasted->setPen(radCurvePen);
	radCrvCasted->setVisible(false);
	radCrvCasted->setBrush(QBrush(QColor(0, 128, 0, 64)));
	radCrvCasted->setName("Missile Proximity Radius");
	radCrvCasted->setAntialiased(true);

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
	delete _leSim;
}

void MainWindow::on_startSimBtn_clicked()
{
	auto leTgt = _leSim->getTarget();
	auto leMsl = _leSim->getMissile();
	
	_leSim->setFileOutputNeededTo(ui->fileOCheckBox->isChecked());
	_leSim->restoreSimState();
	
	leTgt->setCoords(0, ui->distanceSpinBox->value());
	leTgt->setVelocity(0, -ui->tgtSpeedSpinBox->value());
	
	leMsl->setCoords(0, 0);
	leMsl->setVelocity(0, ui->mslSpeedSpinBox->value());

	ui->outputLabel->clear();
	ui->outputLabel->setText("Simulation's running; please wait");
	ui->outputLabel->setStyleSheet("QLabel { color : black; }");

	float tSinceReplot = 0;
	simFinished = false;

	mslX.clear(); mslY.clear();
	tgtX.clear(); tgtY.clear();
	tgtX.append(_leSim->getTarget()->getX());
	tgtY.append(_leSim->getTarget()->getY());
	mslX.append(_leSim->getMissile()->getX());
	mslY.append(_leSim->getMissile()->getY());
	plot();
	
	std::thread simThread([&]{ runSim(); });
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

	if (_leSim->mslWithinTgtHitRadius())
	{
		ui->outputLabel->setText("Simulation's been stopped: the missile has reached the target");
		ui->outputLabel->setStyleSheet("QLabel { color : green; }");
	}
	else if (!_leSim->mslSpeedMoreThanTgtSpeed())
	{
		ui->outputLabel->setText("Simulation's been stopped: the missile's velocity has fallen below the target's");
		ui->outputLabel->setStyleSheet("QLabel { color : red; }");
	}

	plot(true);
}

void MainWindow::on_resetSimBtn_clicked()
{
	mslX.clear(); mslY.clear();
	tgtX.clear(); tgtY.clear();
	ui->outputLabel->clear();
	_leSim->restoreSimState();
	if (radiusCurve) static_cast<QCPCurve*>(radiusCurve)->setVisible(false);
	simFinished = false;

	plot();
}

void MainWindow::plot(bool doFilter)
{
	if (doFilter)
	{
		auto filterData = [&](QVector<double>& keyVec, QVector<double>& valVec)
		{
			auto filterInterval = _leSim->getMissile()->getProxyRadius() * 2;
			
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

	if (doFilter && _leSim)
	{
		auto mslFinalX = mslX.last();
		auto mslFinalY = mslY.last();
		QVector<double> xCoords, yCoords;
		
		xCoords.clear(); yCoords.clear();
		
		for (auto coordX : hitRadX)
			xCoords.append(coordX + mslFinalX);
		
		for (auto coordY : hitRadY)
			yCoords.append(coordY + mslFinalY);
		
		auto radCrvCasted = static_cast<QCPCurve*>(radiusCurve);
		radCrvCasted->setData(xCoords, yCoords);
		radCrvCasted->setVisible(true);
	}

	ui->plot->replot();
	ui->plot->update();
}

void MainWindow::runSim()
{
	while (!simFinished)
	{
		_leSim->iterate();

		tgtX.append(_leSim->getTarget()->getX());
		tgtY.append(_leSim->getTarget()->getY());
		mslX.append(_leSim->getMissile()->getX());
		mslY.append(_leSim->getMissile()->getY());

		simFinished = _leSim->mslWithinTgtHitRadius() || !_leSim->mslSpeedMoreThanTgtSpeed();
	}
}

void MainWindow::prepareHitRadData()
{
	const static double degreesPerStep{ 5 };
	QVector2D radVector{ float(_leSim->getMslProxyRadius()), 0.f };
	double currentAngle{ 0 };

	hitRadX.append(radVector.x());
	hitRadY.append(radVector.y());

	while (currentAngle <= 360)
	{
		currentAngle += degreesPerStep;
		rotateVec(degreesPerStep, radVector, false);

		hitRadX.append(radVector.x());
		hitRadY.append(radVector.y());
	}
}
